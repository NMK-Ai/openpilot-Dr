class DriverStatus():
    def __init__(self, rhd=False, settings=DRIVER_MONITOR_SETTINGS()):
        # init policy settings
        self.settings = settings

        # init driver status
        self.is_rhd_region = rhd
        self.pose = DriverPose(self.settings._POSE_OFFSET_MAX_COUNT)
        self.pose_calibrated = False
        self.blink = DriverBlink()
        self.awareness = 1.
        self.awareness_active = 1.
        self.awareness_passive = 1.
        self.driver_distracted = False
        self.driver_distraction_filter = FirstOrderFilter(0., self.settings._DISTRACTED_FILTER_TS, self.settings._DT_DMON)
        self.face_detected = False
        self.face_partial = False
        self.terminal_alert_cnt = 0
        self.terminal_time = 0
        self.step_change = 0.
        self.active_monitoring_mode = True
        self.is_model_uncertain = False
        self.hi_stds = 0
        self.threshold_pre = self.settings._DISTRACTED_PRE_TIME_TILL_TERMINAL / self.settings._DISTRACTED_TIME
        self.threshold_prompt = self.settings._DISTRACTED_PROMPT_TIME_TILL_TERMINAL / self.settings._DISTRACTED_TIME
        self.slowing_down = False  # To track if gradual stopping is in progress

        self._set_timers(active_monitoring=True)

    def gradual_stop(self, car_interface):
        # Gradually reduce the vehicle's speed until full stop
        current_speed = car_interface.get_speed()
        if current_speed > 0:
            new_speed = max(0, current_speed - 1)  # Decrease speed gradually
            car_interface.set_speed(new_speed)
        else:
            self.slowing_down = False  # Stop the deceleration when speed is 0

    def safe_lane_change_right(self, car_interface, blindspot_data, audio_alert):
        # Check the right blindspot and change lanes if safe
        if not blindspot_data['right']:
            car_interface.set_blinker(False, True)  # Turn on right blinker
            car_interface.set_hazard_lights(True)  # Activate hazard lights
            car_interface.set_steering_angle(15)  # Change steering angle to right
            audio_alert.play('safe_lane_change')  # Play lane change sound alert
        else:
            # If a vehicle is detected in the right blindspot, prevent lane change
            audio_alert.play('blocked_alert')  # Play blocked alert
            car_interface.show_message("Vehicle in right blindspot")  # Display message

    def update(self, events, driver_engaged, ctrl_active, standstill, car_interface, blindspot_data, audio_alert):
        if (driver_engaged and self.awareness > 0) or not ctrl_active:
            # Reset awareness if the driver is in control
            self.awareness = 1.
            self.awareness_active = 1.
            self.awareness_passive = 1.
            return

        driver_attentive = self.driver_distraction_filter.x < 0.37
        awareness_prev = self.awareness

        if (driver_attentive and self.face_detected and self.pose.low_std and self.awareness > 0):
            # Restore awareness when the driver is paying attention
            self.awareness = min(self.awareness + ((self.settings._RECOVERY_FACTOR_MAX-self.settings._RECOVERY_FACTOR_MIN)*(1.-self.awareness)+self.settings._RECOVERY_FACTOR_MIN)*self.step_change, 1.)
            if self.awareness == 1.:
                self.awareness_passive = min(self.awareness_passive + self.step_change, 1.)
            if self.awareness > self.threshold_prompt:
                return

        standstill_exemption = standstill and self.awareness - self.step_change <= self.threshold_prompt
        certainly_distracted = self.driver_distraction_filter.x > 0.63 and self.driver_distracted and self.face_detected
        maybe_distracted = self.hi_stds > self.settings._HI_STD_FALLBACK_TIME or not self.face_detected

        if certainly_distracted or maybe_distracted:
            if not standstill_exemption:
                self.awareness = max(self.awareness - self.step_change, -0.1)

        alert = None
        if self.awareness <= 0.:
            # Final red alert: gradual stop is required
            alert = EventName.driverDistracted if self.active_monitoring_mode else EventName.driverUnresponsive
            self.terminal_time += 1
            if awareness_prev > 0.:
                self.terminal_alert_cnt += 1

            # Activate hazard lights and start gradual deceleration
            car_interface.set_hazard_lights(True)
            self.slowing_down = True

            # Start deceleration
            self.gradual_stop(car_interface)

            # Attempt lane change to the right if safe
            self.safe_lane_change_right(car_interface, blindspot_data, audio_alert)

        elif self.awareness <= self.threshold_prompt:
            # Orange alert
            alert = EventName.promptDriverDistracted if self.active_monitoring_mode else EventName.promptDriverUnresponsive
        elif self.awareness <= self.threshold_pre:
            # Green alert
            alert = EventName.preDriverDistracted if self.active_monitoring_mode else EventName.preDriverUnresponsive

        if alert is not None:
            events.add(alert)

        # Continue decelerating if the vehicle is in gradual stop mode
        if self.slowing_down:
            self.gradual_stop(car_interface)
