/***********************************
   DISPLAY
 ***********************************/
void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_profont11_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  u8g2.setPowerSave(0);
}

bool screenSaverCheck() {
  if (!enableScreenSaver) return false;
  if (brewReady && (millis() >= lastBrewReady + brewReadyWaitPeriod) && millis() >= userActivity + userActivityWaitPeriod) {
    return true;
  } else {
    if (screenSaverOn) {
      u8g2.setPowerSave(0);
      screenSaverOn = false;
      return false;
    }
  }
}

void displaymessage(int activeState, char* displaymessagetext, char* displaymessagetext2) {
  if (Display > 0) {
    unsigned long currentMillisDisplay = millis();
    static char line[10];
    if (currentMillisDisplay >= previousMillisDisplay + intervalDisplay || previousMillisDisplay == 0) {
      previousMillisDisplay = currentMillisDisplay;
      u8g2.clearBuffer();
      u8g2.setBitmapMode(1);
      //u8g2.drawFrame(0, 0, 128, 64);

      if (!screenSaverCheck()) {
        image_flip = !image_flip;
        unsigned int align_right;
        const unsigned int align_right_2digits = LCDWidth - 56;
        const unsigned int align_right_3digits = LCDWidth - 56 - 12;
        const unsigned int align_right_2digits_decimal = LCDWidth - 56 + 28;

        //display icons
        switch (activeState) {
          case 1:
          case 2:
            if (image_flip) {
              u8g2.drawXBMP(0, 0, icon_width, icon_height, coldstart_rotate_bits);
            } else {
              u8g2.drawXBMP(0, 0, icon_width, icon_height, coldstart_bits);
            }
            break;
          case 4: //brew
            if (image_flip) {
              u8g2.drawXBMP(0, 0, icon_width, icon_height, brewing_bits);
            } else {
              u8g2.drawXBMP(0, 0, icon_width, icon_height, brewing_rotate_bits);
            }
            break;
          case 3:
            if (brewReady) {
              if (image_flip) {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, brew_ready_bits);
              } else {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, brew_ready_rotate_bits);
              }
            } else {  //inner zone
              if (image_flip) {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, brew_acceptable_bits);
              } else {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, brew_acceptable_rotate_bits);
              }
            }
            break;
          case 5:
            if (Input >= steamReadyTemp) {  //fallback: if hardware steaming button is used still show steaming icon
              if (image_flip) {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, steam_bits);
              } else {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, steam_rotate_bits);
              }
            } else {
              if (image_flip) {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, outer_zone_bits);
              } else {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, outer_zone_rotate_bits);
              }
            }
            break;
          case 6:  //steaming state (detected via controlAction STEAMING)
            if (Input >= steamReadyTemp) {
              if (image_flip) {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, steam_bits);
              } else {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, steam_rotate_bits);
              }
            } else {
              //TODO create new icons for steam phase
              if (image_flip) {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, outer_zone_bits);
              } else {
                u8g2.drawXBMP(0, 0, icon_width, icon_height, outer_zone_rotate_bits);
              }
            }
            break;
          case 8: // cleaning
            if (image_flip) {
              u8g2.drawXBMP(0, 0, icon_width, icon_height, clean_bits);
            } else {
              u8g2.drawXBMP(0, 0, icon_width, icon_height, clean_bits);
            }
            break;
          default:
            if (MACHINE_TYPE == "rancilio") {
              u8g2.drawXBMP(41, 0, rancilio_logo_width, rancilio_logo_height, rancilio_logo_bits);
            } else if (MACHINE_TYPE == "gaggia") {
              u8g2.drawXBMP(5, 0, gaggia_logo_width, gaggia_logo_height, gaggia_logo_bits);
            } else if (MACHINE_TYPE == "ecm") {
              u8g2.drawXBMP(11, 0, ecm_logo_width, ecm_logo_height, ecm_logo_bits);
            }
            break;
        }

        //display current and target temperature
        if (activeState > 0 && activeState != 4) {
          if (Input - 100 > -FLT_EPSILON) {
            align_right = align_right_3digits;
          } else {
            align_right = align_right_2digits;
          }
          u8g2.setFont(u8g2_font_profont22_tf);
          u8g2.setCursor(align_right, 3);
          u8g2.print(Input, 1);
          u8g2.setFont(u8g2_font_profont10_tf);
          u8g2.print((char)176);
          u8g2.println("C");
          u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
          u8g2.drawGlyph(align_right - 11, 3 + 7, 0x0046);

          //if (Input <= *activeSetPoint + 5 || activeState == 6) { //only show setpoint if we are not steaming
          if (!steaming) {
            if (*activeSetPoint >= 100 ) {
              align_right = align_right_3digits;
            } else {
              align_right = align_right_2digits;
            }
            u8g2.setFont(u8g2_font_profont22_tf);
            u8g2.setCursor(align_right, 20);
            u8g2.print(*activeSetPoint, 1);
            u8g2.setFont(u8g2_font_profont10_tf);
            u8g2.print((char)176);
            u8g2.println("C");
            u8g2.setFont(u8g2_font_open_iconic_other_1x_t);
            u8g2.drawGlyph(align_right - 11 , 20 + 7, 0x047);
          }
        } else if (activeState == 4) {
          totalbrewtime = (OnlyPID ? brewtime : preinfusion + preinfusionpause + brewtime) * 1000;
          align_right = align_right_2digits_decimal;
          u8g2.setFont(u8g2_font_profont22_tf);
          u8g2.setCursor(align_right, 3);
          if (bezugsZeit < 10000) u8g2.print("0");
          // TODO: Use print(u8x8_u8toa(value, digits)) or print(u8x8_u16toa(value, digits)) to print numbers with constant width (numbers are prefixed with 0 if required).
          u8g2.print(bezugsZeit / 1000);
          u8g2.setFont(u8g2_font_profont10_tf);
          u8g2.println("s");
          if (totalbrewtime > 0) {
            u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
            u8g2.drawGlyph(align_right - 11, 3 + 7, 0x0046);
            u8g2.setFont(u8g2_font_profont22_tf);
            u8g2.setCursor(align_right, 20);
            u8g2.print(totalbrewtime / 1000);
            u8g2.setFont(u8g2_font_profont10_tf);
            u8g2.println("s");
            u8g2.setFont(u8g2_font_open_iconic_other_1x_t);
            u8g2.drawGlyph(align_right - 11 , 20 + 7, 0x047);
          }
        }
      } else {
        static unsigned int screen_saver_x_pos = 41;
        static bool screen_saver_direction_right = true;
        const int unsigned screen_saver_step = 4;
        unsigned int logo_width = icon_width;
        if ( enableScreenSaver == 3 && MACHINE_TYPE == "gaggia") {
          logo_width = 125;  //hack which will result in logo only moving left
          screen_saver_x_pos = 5;
        } if ( enableScreenSaver == 3 && MACHINE_TYPE == "ecm") {
          logo_width = 125;  //hack which will result in logo only moving left
          screen_saver_x_pos = 11;
        }
        if (screen_saver_direction_right) {
          if (screen_saver_x_pos + screen_saver_step <= LCDWidth - logo_width ) {
            screen_saver_x_pos += screen_saver_step;
          } else {
            screen_saver_x_pos -= screen_saver_step;
            screen_saver_direction_right = false;
          }
        } else {
          if (screen_saver_x_pos >= screen_saver_step ) {
            screen_saver_x_pos -= screen_saver_step;
          } else {
            screen_saver_x_pos += screen_saver_step;
            screen_saver_direction_right = true;
          }
        }
        if ( enableScreenSaver == 1 ) {
          u8g2.setPowerSave(1);
        } else if ( enableScreenSaver == 2 ) {
          u8g2.drawXBMP(screen_saver_x_pos, 0, icon_width, icon_height, brew_ready_bits);
        } else if ( enableScreenSaver == 3 ) {
          if (MACHINE_TYPE == "rancilio") {
            u8g2.drawXBMP(screen_saver_x_pos, 0, rancilio_logo_width, rancilio_logo_height, rancilio_logo_bits);
          } else if (MACHINE_TYPE == "gaggia") {
            u8g2.drawXBMP(screen_saver_x_pos, 0, gaggia_logo_width, gaggia_logo_height, gaggia_logo_bits);  //TODO fix
          } else if (MACHINE_TYPE == "ecm") {
            u8g2.drawXBMP(screen_saver_x_pos, 0, ecm_logo_width, ecm_logo_height, ecm_logo_bits);  //TODO fix
          }
        }
        screenSaverOn = true;
      }

      //power-off timer
#if (ENABLE_POWER_OFF_COUNTDOWN > 0)
      const unsigned int powerOffCountDownStart = 300;
      const unsigned int align_right_countdown_min = LCDWidth - 52 ;
      const unsigned int align_right_countdown_sec = LCDWidth - 52 + 20;

      if (power_off_timer <= powerOffCountDownStart && !brewing && displaymessagetext == "" && displaymessagetext2 == "" ) {
        u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
        u8g2.drawGlyph(align_right_countdown_min - 15, 37 + 7, 0x004e);
        u8g2.setFont(u8g2_font_profont22_tf);
        u8g2.setCursor(align_right_countdown_min, 37);
        snprintf(line, sizeof(line), "%d", int(power_off_timer / 60));
        u8g2.print(line);
        u8g2.setFont(u8g2_font_profont10_tf);
        u8g2.println("m");
        u8g2.setFont(u8g2_font_profont22_tf);
        u8g2.setCursor(align_right_countdown_sec, 37);
        snprintf(line, sizeof(line), "%0.2d", int(power_off_timer % 60));
        u8g2.print(line);
        u8g2.setCursor(align_right_countdown_sec + 23, 37);
        u8g2.setFont(u8g2_font_profont10_tf);
        u8g2.println(" S");
      }
#endif

      //(optional) add 2 text lines
      u8g2.setFont(u8g2_font_profont11_tf);
      u8g2.setCursor(ALIGN_CENTER(displaymessagetext), 44);  // 9 pixel space between lines
      u8g2.print(displaymessagetext);
      u8g2.setCursor(ALIGN_CENTER(displaymessagetext2), 53);
      u8g2.print(displaymessagetext2);

      //add status icons
#if (ENABLE_FAILURE_STATUS_ICONS == 1)
      if (image_flip) {
        byte icon_y = 64 - (status_icon_height - 1);
        byte icon_counter = 0;
        if ((!force_offline && !wifi_working()) || (force_offline && !FORCE_OFFLINE)) {
          u8g2.drawXBMP(0, 64 - status_icon_height + 1, status_icon_width, status_icon_height, wifi_not_ok_bits);
          u8g2.drawXBMP(icon_counter * (status_icon_width - 1) , icon_y, status_icon_width, status_icon_height, wifi_not_ok_bits);
          icon_counter++;
        }
        if (BLYNK_ENABLE && !blynk_working()) {
          u8g2.drawXBMP(icon_counter * (status_icon_width - 1), icon_y, status_icon_width, status_icon_height, blynk_not_ok_bits);
          icon_counter++;
        }
        if (MQTT_ENABLE && !mqtt_working()) {
          u8g2.drawXBMP(icon_counter * (status_icon_width - 1), icon_y, status_icon_width, status_icon_height, mqtt_not_ok_bits);
          icon_counter++;
        }
      }
#endif

      u8g2.sendBuffer();

    }
  }
}
