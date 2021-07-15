/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#ifndef WIFI_TASK_H
#define WIFI_TASK_H

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

void start_wifi(void *args);
void stop_wifi();

#endif