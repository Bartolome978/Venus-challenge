#include <libpynq.h>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/*
 * TU/e 5EID0 :: PYNQ-side MQTT<=>UART bridge helper.
 *
 * The ESP32 is assumed to already run the course bridge firmware:
 * - anything written by the PYNQ to UART is published to
 *   /PYNQBRIDGE/{MODULE}/SEND
 * - anything published to /PYNQBRIDGE/{MODULE}/RECV is forwarded by the ESP32
 *   over UART to the PYNQ
 *
 * UART format in both directions:
 * - UTF-8 / ASCII text
 * - newline ('\n') terminated
 *
 * Interface:
 * - TX: type a terminal line and press Enter to publish it through the ESP32
 * - RX: UART lines are reconstructed incrementally and printed
 *
 * Build example:
 *   make CFLAGS_EXTRA='-DMODULE_NUMBER=\"74\"'
 */

#ifndef MODULE_NUMBER
#define MODULE_NUMBER "74"
#endif

#define MQTT_HOSTNAME "mqtt.ics.ele.tue.nl"
#define MQTT_TOPIC_ROOT "/PYNQBRIDGE"

#define UART_RX_PIN IO_AR0
#define UART_TX_PIN IO_AR1

#define MAX_UART_LINE_BYTES 512U
#define MAX_STDIN_LINE_BYTES 512U
#define POLL_DELAY_MS 10
#define UART_PARTIAL_FLUSH_MS 50U

typedef struct {
  char line[MAX_UART_LINE_BYTES + 1U];
  size_t line_length;
  bool discarding_line;
  uint64_t last_byte_ms;
} uart_line_rx_t;

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signum) {
  (void)signum;
  keep_running = 0;
}

static void reset_rx_state(uart_line_rx_t *rx) {
  rx->line_length = 0U;
  rx->discarding_line = false;
  rx->last_byte_ms = 0U;
}

static uint64_t monotonic_ms(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    return 0U;
  }

  return ((uint64_t)ts.tv_sec * 1000U) + ((uint64_t)ts.tv_nsec / 1000000U);
}

static uint32_t decode_le_u32(const uint8_t bytes[4]) {
  return ((uint32_t)bytes[0]) | ((uint32_t)bytes[1] << 8) |
         ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static void print_laptop_examples(void) {
  printf("ESP32 bridge host: %s\n", MQTT_HOSTNAME);
  printf("ESP32 publish topic: %s/%s/SEND\n", MQTT_TOPIC_ROOT, MODULE_NUMBER);
  printf("ESP32 subscribe topic: %s/%s/RECV\n", MQTT_TOPIC_ROOT, MODULE_NUMBER);
  printf("Laptop receive example:\n");
  printf("  mosquitto_sub -h %s -t %s/%s/SEND -u <USER> -P <PASSWORD>\n",
         MQTT_HOSTNAME, MQTT_TOPIC_ROOT, MODULE_NUMBER);
  printf("Laptop send example:\n");
  printf("  mosquitto_pub -h %s -t %s/%s/RECV -u <USER> -P <PASSWORD> -m "
         "\"hello from laptop\"\n",
         MQTT_HOSTNAME, MQTT_TOPIC_ROOT, MODULE_NUMBER);
  fflush(stdout);
}

static void send_uart_line(const uint8_t *payload, uint32_t payload_length) {
  for (uint32_t i = 0; i < payload_length; ++i) {
    uart_send(UART0, payload[i]);
  }
  uart_send(UART0, (uint8_t)'\n');
}

static void handle_complete_uart_line(const uart_line_rx_t *rx) {
  const uint8_t *payload = (const uint8_t *)rx->line;
  size_t payload_length = rx->line_length;
  bool had_length_prefix = false;

  if (rx->line_length >= 4U) {
    const uint32_t framed_length = decode_le_u32((const uint8_t *)rx->line);
    if (framed_length == rx->line_length - 4U) {
      payload = (const uint8_t *)rx->line + 4U;
      payload_length = rx->line_length - 4U;
      had_length_prefix = true;
    }
  }

  printf(">> Incoming from MQTT %s/%s/RECV: %zu byte(s)\n", MQTT_TOPIC_ROOT,
         MODULE_NUMBER, payload_length);
  if (had_length_prefix) {
    printf("   Format: length-prefixed UART frame detected\n");
  }
  if (payload_length == 0U) {
    printf("   Payload: <empty>\n");
  } else {
    printf("   Payload: ");
    fwrite(payload, 1U, payload_length, stdout);
    printf("\n");
  }
  fflush(stdout);
}

static void flush_partial_uart_line_if_idle(uart_line_rx_t *rx) {
  if (rx->discarding_line || rx->line_length == 0U) {
    return;
  }

  const uint64_t now_ms = monotonic_ms();
  if (now_ms == 0U || rx->last_byte_ms == 0U ||
      now_ms - rx->last_byte_ms < UART_PARTIAL_FLUSH_MS) {
    return;
  }

  rx->line[rx->line_length] = '\0';
  handle_complete_uart_line(rx);
  reset_rx_state(rx);
}

static void poll_uart_lines(uart_line_rx_t *rx) {
  while (uart_has_data(UART0)) {
    const char c = (char)uart_recv(UART0);
    rx->last_byte_ms = monotonic_ms();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (rx->discarding_line) {
        printf("!! Dropped oversized UART line (max %u bytes)\n",
               MAX_UART_LINE_BYTES);
        fflush(stdout);
        reset_rx_state(rx);
      } else {
        rx->line[rx->line_length] = '\0';
        handle_complete_uart_line(rx);
        reset_rx_state(rx);
      }
      continue;
    }

    if (rx->discarding_line) {
      continue;
    }

    if (rx->line_length < MAX_UART_LINE_BYTES) {
      rx->line[rx->line_length++] = c;
    } else {
      rx->discarding_line = true;
    }
  }
}

static void flush_stdin_line(uint8_t *line_buffer, size_t *line_length) {
  if (*line_length == 0U) {
    return;
  }

  printf("<< Outgoing UART line: %zu byte(s)\n", *line_length);
  printf("   Payload: %.*s\n", (int)*line_length, (const char *)line_buffer);
  fflush(stdout);

  send_uart_line(line_buffer, (uint32_t)(*line_length));
  *line_length = 0U;
}

static void poll_terminal_and_forward(void) {
  static uint8_t line_buffer[MAX_STDIN_LINE_BYTES];
  static size_t line_length = 0U;
  static bool discarding_long_line = false;
  uint8_t read_buffer[128];

  const ssize_t bytes_read = read(STDIN_FILENO, read_buffer, sizeof(read_buffer));
  if (bytes_read < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      perror("read");
      keep_running = 0;
    }
    return;
  }

  if (bytes_read == 0) {
    return;
  }

  for (ssize_t i = 0; i < bytes_read; ++i) {
    const uint8_t c = read_buffer[i];

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (discarding_long_line) {
        printf("!! Dropped oversized terminal line (max %u bytes)\n",
               MAX_STDIN_LINE_BYTES);
        fflush(stdout);
        discarding_long_line = false;
        line_length = 0U;
      } else {
        flush_stdin_line(line_buffer, &line_length);
      }
      continue;
    }

    if (discarding_long_line) {
      continue;
    }

    if (line_length < MAX_STDIN_LINE_BYTES) {
      line_buffer[line_length++] = c;
    } else {
      discarding_long_line = true;
    }
  }
}

static bool set_stdin_nonblocking(void) {
  const int current_flags = fcntl(STDIN_FILENO, F_GETFL);
  if (current_flags < 0) {
    perror("fcntl(F_GETFL)");
    return false;
  }

  if (fcntl(STDIN_FILENO, F_SETFL, current_flags | O_NONBLOCK) < 0) {
    perror("fcntl(F_SETFL)");
    return false;
  }

  return true;
}

int main(void) {
  uart_line_rx_t rx;
  reset_rx_state(&rx);

  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  pynq_init();
  switchbox_init();
  switchbox_set_pin(UART_RX_PIN, SWB_UART0_RX);
  switchbox_set_pin(UART_TX_PIN, SWB_UART0_TX);
  uart_init(UART0);
  uart_reset_fifos(UART0);

  if (!set_stdin_nonblocking()) {
    uart_reset_fifos(UART0);
    uart_destroy(UART0);
    pynq_destroy();
    return EXIT_FAILURE;
  }

  printf("PYNQ UART<=>MQTT bridge helper starting.\n");
  printf("Type a line and press Enter to publish through the ESP32.\n");
  print_laptop_examples();

  while (keep_running) {
    poll_terminal_and_forward();
    poll_uart_lines(&rx);
    flush_partial_uart_line_if_idle(&rx);
    sleep_msec(POLL_DELAY_MS);
  }

  printf("Stopping bridge.\n");
  fflush(stdout);
  uart_reset_fifos(UART0);
  uart_destroy(UART0);
  pynq_destroy();
  return EXIT_SUCCESS;
}
