void isrFaultJ0(void) {
  //Arriving here on FALLING interrupt on J0 driver fault line
  cli();
  Serial.println("ERR: Joint 0 drive failure");
  j0fault = true;
  disableMotors();
  sei();
}

void isrUpdateJ1ac(void) {
  // Arriving here on CHANGE interrupt on J1 quadrature A signal
  cli();
  if (digitalReadFast(j1qaPin)) {
    // Rising A
    if (!digitalReadFast(j1qbPin)) {
      j1pos++; j1forward = true;
    } else {
      j1pos--; j1forward = false;
    }
  } else {
    // Falling A
    if (digitalReadFast(j1qbPin)) {
      j1pos++; j1forward = true;
    } else {
      j1pos--; j1forward = false;
    }
  }
  j1ticktime = 0.85*j1ticktime + 0.15*(micros()-lastj1tick);
  lastj1tick = micros();
  sei();
}

void isrUpdateJ1bc(void) {
  // Arriving here on CHANGE interrupt on J1 quadrature B signal
  cli();
  if (digitalReadFast(j1qbPin)) {
    // Rising B
    if (digitalReadFast(j1qaPin)) {
      j1pos++; j1forward = true;
    } else {
      j1pos--; j1forward = false;
    }
  } else {
    // Falling B
    if (!digitalReadFast(j1qaPin)) {
      j1pos++; j1forward = true;
    } else {
      j1pos--; j1forward = false;
    }
  }
  j1ticktime = 0.85*j1ticktime + 0.15*(micros()-lastj1tick);
  lastj1tick = micros();
  sei();
}

void isrUpdateJ2ar(void) {
  //Arriving here on RISING interrupt on J2 quadrature A signal
  cli();
  if (!digitalReadFast(j2qbPin)) {
    j2pos++;
    j2forward = true;
  } else {
    j2pos--;
    j2forward = false;
  }
  j2ticktime = 0.85*j2ticktime + 0.15*(micros()-lastj2tick);
  lastj2tick = micros();
  sei();
}

