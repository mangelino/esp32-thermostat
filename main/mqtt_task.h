
#ifndef MQTT_TASK_H
#define MQTT_TASK_H

typedef void (*message_handler_fptr_t)(char *topic, char *data);

void mqtt_register_handler(message_handler_fptr_t handler);
void start_mqtt(void *args);
void mqtt_publish(const  char *topic, char* data, int qos);
void mqtt_disconnect();
bool is_mqtt_connected();

#endif