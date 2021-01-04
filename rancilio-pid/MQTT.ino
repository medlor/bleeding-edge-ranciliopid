/********************************************************
  MQTT
*****************************************************/
bool almostEqual(float a, float b) {
    return fabs(a - b) <= FLT_EPSILON;
}
char* bool2string(bool in) {
  if (in) {
    return "1";
  } else {
    return "0";
  }
}
char number2string_double[22];
char* number2string(double in) {
  snprintf(number2string_double, sizeof(number2string_double), "%0.2f", in);
  return number2string_double;
}
char number2string_float[22];
char* number2string(float in) {
  snprintf(number2string_float, sizeof(number2string_float), "%0.2f", in);
  return number2string_float;
}
char number2string_int[22];
char* number2string(int in) {
  snprintf(number2string_int, sizeof(number2string_int), "%d", in);
  return number2string_int;
}
char number2string_uint[22];
char* number2string(unsigned int in) {
  snprintf(number2string_uint, sizeof(number2string_uint), "%u", in);
  return number2string_uint;
}

char* mqtt_build_topic(char* reading) {
  char* topic = (char *) malloc(sizeof(char) * 256);
  snprintf(topic, sizeof(topic), "%s%s/%s", mqtt_topic_prefix, hostname, reading);
  return topic;
}

/* ------------------------------ */
#if (MQTT_ENABLE == 0)  //MQTT Disabled
bool mqtt_publish(char* reading, char* payload) { return true; }
bool mqtt_reconnect(bool force_connect = false) { return true; }
bool mqtt_working() { return false; }

/* ------------------------------ */
#elif (MQTT_ENABLE == 1)  //MQTT Client
bool mqtt_working() {
  return ((MQTT_ENABLE >0) && (wifi_working()) && (mqtt_client.connected()));
}

bool mqtt_publish(char* reading, char* payload) {
  if (!MQTT_ENABLE || force_offline || mqtt_disabled_temporary) return true;
  if (!mqtt_working()) { return false; }
  char topic[MQTT_MAX_PUBLISH_SIZE];
  snprintf(topic, MQTT_MAX_PUBLISH_SIZE, "%s%s/%s", mqtt_topic_prefix, hostname, reading);

  if (strlen(topic) + strlen(payload) >= MQTT_MAX_PUBLISH_SIZE) {
    ERROR_print("mqtt_publish() wants to send too much data (len=%u)\n", strlen(topic) + strlen(payload));
    return false;
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis > mqtt_dontPublishUntilTime) {
      bool ret = mqtt_client.publish(topic, payload, mqtt_flag_retained);
      if (ret == false) {
        mqtt_dontPublishUntilTime = millis() + mqtt_dontPublishBackoffTime;
        ERROR_print("Error on publish. Wont publish the next %ul ms\n", mqtt_dontPublishBackoffTime);
        mqtt_client.disconnect();
      }
      return ret;
    } else { //TODO test this code block later (faking an error)
      ERROR_print("Data not published (still for the next %ul ms)\n", mqtt_dontPublishUntilTime - currentMillis);
      return false;
    }
  }
}

bool mqtt_reconnect(bool force_connect = false) {
  if (!MQTT_ENABLE || force_offline || mqtt_disabled_temporary || mqtt_working() || in_sensitive_phase() ) return true;
  espClient.setTimeout(2000); // set timeout for mqtt connect()/write() to 2 seconds (default 5 seconds).
  unsigned long now = millis();
  if ( force_connect || ((now > mqtt_lastReconnectAttemptTime + (mqtt_reconnect_incremental_backoff * (mqtt_reconnectAttempts))) && now > all_services_lastReconnectAttemptTime + all_services_min_reconnect_interval)) {
    mqtt_lastReconnectAttemptTime = now;
    all_services_lastReconnectAttemptTime = now;
    DEBUG_print("Connecting to mqtt ...\n");
    if (mqtt_client.connect(hostname, mqtt_username, mqtt_password, topic_will, 0, 0, "unexpected exit") == true) {
      DEBUG_print("Connected to mqtt server\n");
      mqtt_publish("events", "Connected to mqtt server");
      if (!mqtt_client.subscribe(topic_set)) { ERROR_print("Cannot subscribe to topic\n"); }
      mqtt_lastReconnectAttemptTime = 0;
      mqtt_reconnectAttempts = 0;
    } else {
      DEBUG_print("Cannot connect to mqtt server (consecutive failures=#%u)\n", mqtt_reconnectAttempts);
      if (mqtt_reconnectAttempts < mqtt_max_incremental_backoff) {
        mqtt_reconnectAttempts++;
      }
    }
  }
  return mqtt_client.connected();
}

void mqtt_callback(char* topic, byte* data, unsigned int length) {
  //DEBUG_print("Message arrived [%s]: %s\n", topic, (const char *)data);
  char topic_str[255];
  os_memcpy(topic_str, topic, sizeof(topic_str));
  topic_str[255] = '\0';
  char data_str[length+1];
  os_memcpy(data_str, data, length);
  data_str[length] = '\0';
  //DEBUG_print("MQTT: %s = %s\n", topic_str, data_str);
  mqtt_parse(topic_str, data_str);
}

/* ------------------------------ */
#elif (MQTT_ENABLE == 2)
bool mqtt_working() {
  return ((MQTT_ENABLE >0) && (wifi_working()));
}

bool mqtt_publish(char* reading, char* payload) {
  if (!MQTT_ENABLE || force_offline || mqtt_disabled_temporary) return true;
  char topic[MQTT_MAX_PUBLISH_SIZE];
  snprintf(topic, MQTT_MAX_PUBLISH_SIZE, "%s%s/%s", mqtt_topic_prefix, hostname, reading);
  if (!mqtt_working()) { return false; }
  if (strlen(topic) + strlen(payload) >= MQTT_MAX_PUBLISH_SIZE) {
    ERROR_print("mqtt_publish() wants to send too much data (len=%u)\n", strlen(topic) + strlen(payload));
    return false;
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis > mqtt_dontPublishUntilTime) {
      bool ret = MQTT_local_publish((unsigned char*)&topic, (unsigned char*)payload, strlen(payload), 1, 1);
      if (ret == false) {
        mqtt_dontPublishUntilTime = millis() + mqtt_dontPublishBackoffTime;
         ERROR_print("Error on publish. Wont publish the next %ul ms\n", mqtt_dontPublishBackoffTime);
        //mqtt_client.disconnect();
        //MQTT_server_cleanupClientCons();
      }
      return ret;
    } else { //TODO test this code block later (faking an error)
      ERROR_print("Data not published (still for the next %ul ms)\n", mqtt_dontPublishUntilTime - currentMillis);
      return false;
    }
  }
}

bool mqtt_reconnect(bool force_connect = false) { return true; }

void mqtt_callback(uint32_t *client, const char* topic, uint32_t topic_len, const char *data, uint32_t length) {
  char topic_str[topic_len+1];
  os_memcpy(topic_str, topic, topic_len);
  topic_str[topic_len] = '\0';
  char data_str[length+1];
  os_memcpy(data_str, data, length);
  data_str[length] = '\0';
  //DEBUG_print("MQTT: %s = %s\n", topic_str, data_str);
  mqtt_parse(topic_str, data_str);
}

#endif

void mqtt_parse(char* topic_str, char* data_str) {
  char topic_pattern[255];
  char configVar[120];
  char cmd[64];
  double data_double;
  int data_int;

  //DEBUG_print("mqtt_parse(%s, %s)\n", topic_str, data_str);
  snprintf(topic_pattern, sizeof(topic_pattern), "%s%s/%%[^\\/]/%%[^\\/]", mqtt_topic_prefix, hostname);
  //DEBUG_print("topic_pattern=%s\n",topic_pattern);
  if ( (sscanf( topic_str, topic_pattern , &configVar, &cmd) != 2) || (strcmp(cmd, "set") != 0) ) {
    //DEBUG_print("Ignoring topic (%s)\n", topic_str);
    return;
  }
  if (strcmp(configVar, "brewtime") == 0) {
    if (persist_setting("brewtime", &brewtime, data_str)) {
      Blynk.virtualWrite(V8, String(brewtime, 1));
    }
    return;
  }
  if (strcmp(configVar, "starttemp") == 0) {
    if (persist_setting("starttemp", &starttemp, data_str)) {
      Blynk.virtualWrite(V12, String(starttemp, 1));
    }
    return;
  }
  if (strcmp(configVar, "setPoint") == 0) {
    if (persist_setting("setPoint", &setPoint, data_str)) {
      Blynk.virtualWrite(V7, String(setPoint, 1));
    }
    return;
  }
  if (strcmp(configVar, "preinfusion") == 0) {
    if (persist_setting("preinfusion", &preinfusion, data_str)) {
      Blynk.virtualWrite(V9, String(preinfusion, 1));
    }
    return;
  }
  if (strcmp(configVar, "preinfusionpause") == 0) {
    if (persist_setting("preinfusionpause", &preinfusionpause, data_str)) {
      Blynk.virtualWrite(V10, String(preinfusionpause, 1));
    }
    return;
  }
  if (strcmp(configVar, "pidON") == 0) {
    if (persist_setting("pidON", &pidON, data_str)) {
      Blynk.virtualWrite(V13, String(pidON, 1));
    }
    return;
  }
  if (strcmp(configVar, "brewDetectionSensitivity") == 0) {
    if (persist_setting("brewDetectionSensitivity", &brewDetectionSensitivity, data_str)) {
      Blynk.virtualWrite(V34, String(brewDetectionSensitivity, 1));
    }
    return;
  }
  if (strcmp(configVar, "brewDetectionPower") == 0) {
    if (persist_setting("brewDetectionPower", &brewDetectionPower, data_str)) {
      Blynk.virtualWrite(V36, String(brewDetectionPower, 1));
    }
    return;
  }
  if (strcmp(configVar, "steadyPower") == 0) {
    if (persist_setting("steadyPower", &steadyPower, data_str)) {
      //Blynk.virtualWrite(V41, String(steadyPower, 1));  //handled every few seconds by another func
    }
    return;
  }
  if (strcmp(configVar, "steadyPowerOffset") == 0) {
    if (persist_setting("steadyPowerOffset", &steadyPowerOffset, data_str)) {
      Blynk.virtualWrite(V42, String(steadyPowerOffset, 1));
    }
    return;
  }
  if (strcmp(configVar, "steadyPowerOffsetTime") == 0) {
    if (persist_setting("steadyPowerOffsetTime", &steadyPowerOffsetTime, data_str)) {
      Blynk.virtualWrite(V43, String(steadyPowerOffsetTime, 1));
    }
    return;
  }
  if (strcmp(configVar, "aggKp") == 0) {
    if (persist_setting("aggKp", &aggKp, data_str)) {
      Blynk.virtualWrite(V4, String(aggKp, 1));
    }
    return;
  }
  if (strcmp(configVar, "aggTn") == 0) {
    if (persist_setting("aggTn", &aggTn, data_str)) {
      Blynk.virtualWrite(V5, String(aggTn, 1));
    }
    return;
  }
  if (strcmp(configVar, "aggTv") == 0) {
    if (persist_setting("aggTv", &aggTv, data_str)) {
      Blynk.virtualWrite(V6, String(aggTv, 1));
    }
    return;
  }
  if (strcmp(configVar, "aggoKp") == 0) {
    if (persist_setting("aggoKp", &aggoKp, data_str)) {
      Blynk.virtualWrite(V30, String(aggoKp, 1));
    }
    return;
  }
  if (strcmp(configVar, "aggoTn") == 0) {
    if (persist_setting("aggoTn", &aggoTn, data_str)) {
      Blynk.virtualWrite(V31, String(aggoTn, 1));
    }
    return;
  }
  if (strcmp(configVar, "aggoTv") == 0) {
    if (persist_setting("aggoTv", &aggoTv, data_str)) {
      Blynk.virtualWrite(V32, String(aggoTv, 1));
    }
    return;
  }
}

boolean persist_setting(char* type, double* value, char* data_str) {
    double data_double;
    sscanf(data_str, "%lf", &data_double);
    if (strcmp(type, "steadyPower") == 0 && almostEqual(data_double, steadyPowerMQTTDisableUpdateUntilProcessed)) {
      steadyPowerMQTTDisableUpdateUntilProcessed = 0;
      steadyPowerMQTTDisableUpdateUntilProcessedTime = 0;
    }
    if (!almostEqual(data_double, *value)) {
      //DEBUG_print("setting %s=%s (=%0.2f) (prev=%.2f)\n", type, data_str, data_double, *value);
      *value = data_double;
      if (strcmp(type, "steadyPower") == 0) {
        steadyPowerSaved = *value; //prevent an additional mqtt "/set" call
      }
      mqtt_publish(type, data_str);
      force_eeprom_sync = millis();
      return true;
    }
    return false;
}

boolean persist_setting(char* type, int* value, char* data_str) {
    int data_int;
    sscanf(data_str, "%d", &data_int);
    if (data_int != *value) {
      //DEBUG_print("setting %s=%s (=%d) (prev=%d)\n", type, data_str, data_int, *value);
      *value = data_int;
      mqtt_publish(type, data_str);
      force_eeprom_sync = millis();
      return true;
    }
    return false;
}

void mqtt_publish_settings() {
  mqtt_publish("brewtime/set", number2string(brewtime));
  mqtt_publish("starttemp/set", number2string(starttemp));
  mqtt_publish("setPoint/set", number2string(setPoint));
  mqtt_publish("preinfusion/set", number2string(preinfusion));
  mqtt_publish("preinfusionpause/set", number2string(preinfusionpause));
  mqtt_publish("pidON/set", number2string(pidON));
  mqtt_publish("brewDetectionSensitivity/set", number2string(brewDetectionSensitivity));
  mqtt_publish("brewDetectionPower/set", number2string(brewDetectionPower));
  mqtt_publish("steadyPower/set", number2string(steadyPower));
  mqtt_publish("steadyPowerOffset/set", number2string(steadyPowerOffset));
  mqtt_publish("steadyPowerOffsetTime/set", number2string(steadyPowerOffsetTime));
  mqtt_publish("aggKp/set", number2string(aggKp));
  mqtt_publish("aggTn/set", number2string(aggTn));
  mqtt_publish("aggTv/set", number2string(aggTv));
  mqtt_publish("aggoKp/set", number2string(aggoKp));
  mqtt_publish("aggoTn/set", number2string(aggoTn));
  mqtt_publish("aggoTv/set", number2string(aggoTv));
}
