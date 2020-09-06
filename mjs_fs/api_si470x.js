let Si470X = {

  // ## **`Si470X.get()`**
  // Get First Si470X device handle. Return value: opaque pointer.
  get: ffi('void *mgos_si470x_get_global(void)'),

  // ## **`Si470X.close(handle)`**
  // Close Si470X handle. Return value: none.
  close: ffi('void mgos_si470x_delete(void *)'),

  // ## **`Si470X.powerOn(handle)`**
  // Power on the tuner.
  // Return value: success, true/false.
  powerOn: ffi('bool mgos_si470x_power_on(void *)'),

  // ## **`Si470X.powerOff(handle)`**
  // Power off the tuner.
  // Return value: success, true/false.
  powerOff: ffi('bool mgos_si470x_power_off(void *)'),

  // ## **`Si470X.isOn(handle)`**
  // Is the tuner on?
  // Return value: true/false.
  isOn: ffi('bool mgos_si470x_is_on(void *)'),

  // ## **`Si470X.setFrequency(handle, int)`**
  // Set the tuner frequency (in Hz).
  // Return value: success, true/false.
  setFrequency: ffi('bool mgos_si470x_set_frequency(void *, int)'),

  // ## **`Si470X.setVolume(handle, int)`**
  // Set the tuner volume: 0..15.
  // Return value: success, true/false.
  setVolume: ffi('bool mgos_si470x_set_volume(void *, int)'),

  // ## **`Si470X.setMute(handle, bool)`**
  // Set the mute: enabled/disabled.
  // Return value: success, true/false.
  setMute: ffi('bool mgos_si470x_set_mute(void *, bool)'),

  // ## **`Si470X.setSoftMute(handle, bool)`**
  // Set the mute: enabled/disabled.
  // Return value: success, true/false.
  setSoftMute: ffi('bool mgos_si470x_set_soft_mute(void *, bool)'),
};
