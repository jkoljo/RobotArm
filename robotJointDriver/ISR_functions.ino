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
      j1pos++; j1fwtick = true;
    } else {
      j1pos--; j1fwtick = false;
    }
  } else {
    // Falling A
    if (digitalReadFast(j1qbPin)) {
      j1pos++; j1fwtick = true;
    } else {
      j1pos--; j1fwtick = false;
    }
  }
  j1ticktime = 0.85*j1ticktime + 0.15*(micros()-lastj1tick);
  lastj1tick = micros();

  if (j1fwtick && (j1fwcount < 7)) j1fwcount++;
  else if (!j1fwtick && (j1fwcount > 0)) j1fwcount--; 
  sei();
}

void isrUpdateJ1bc(void) {
  // Arriving here on CHANGE interrupt on J1 quadrature B signal
  cli();
  if (digitalReadFast(j1qbPin)) {
    // Rising B
    if (digitalReadFast(j1qaPin)) {
      j1pos++; j1fwtick = true;
    } else {
      j1pos--; j1fwtick = false;
    }
  } else {
    // Falling B
    if (!digitalReadFast(j1qaPin)) {
      j1pos++; j1fwtick = true;
    } else {
      j1pos--; j1fwtick = false;
    }
  }
  j1ticktime = 0.85*j1ticktime + 0.15*(micros()-lastj1tick);
  lastj1tick = micros();

  if (j1fwtick && (j1fwcount < 7)) j1fwcount++;
  else if (!j1fwtick && (j1fwcount > 0)) j1fwcount--; 
  sei();
}

void isrUpdateJ2ar(void) {
  //Arriving here on RISING interrupt on J2 quadrature A signal
  cli();
  if (!digitalReadFast(j2qbPin)) {
    j2pos++;
    j2fwtick = true;
  } else {
    j2pos--;
    j2fwtick = false;
  }
  j2ticktime = 0.8*j2ticktime + 0.2*(micros()-lastj2tick);
  lastj2tick = micros();
  
  if (j2fwtick && (j2fwcount < 7)) j2fwcount++;
  else if (!j2fwtick && (j2fwcount > 0)) j2fwcount--;   
  sei();
}

