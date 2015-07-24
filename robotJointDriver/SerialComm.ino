#define BUFSIZE 32

// Serial comms functions and variables
boolean code_seen(char);
int code_value(void);
boolean getBlock(char, int&);
char cmdBuffer[BUFSIZE + 1]; // + 1 to make space for NULL
char *strchr_pointer;


boolean code_seen(char code) {
  strchr_pointer = strchr(cmdBuffer, code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

int code_value() {
  return (int)strtol(&cmdBuffer[strchr_pointer - cmdBuffer + 1], NULL, 0);
}

void serialRead() {
  while (Serial.available()) {
    char inChar = Serial.read();
    
    if ((inChar == '\n') || (inChar == '\c')) {
      stringComplete = true;
      cmdBuffer[bufferLength] = '\0';
      return; //Return to handle the received command
    } else if (bufferLength < BUFSIZE) {
      cmdBuffer[bufferLength++] = inChar;
    } else {
      //Buffer full, discard received chars
      Serial.println("ERR: Buffer full");
      while ((inChar != '\n') || (inChar != '\c')) {
        inChar = Serial.read();
      }
      bufferLength = 0;
      return;
    }
  }
}

boolean getBlock(char block, float &result) {
  if(code_seen(block) /*&& (code_value() >= 0)*/) { //Allow negative speed
    result = code_value();
    return true;
  } else {
    Serial.print("ERR: No valid ");
    Serial.print(block);
    Serial.println(" block found");
    stringComplete = false;
    bufferLength = 0;
    return false;
  }
}

 /* //Debug for input commands
    Serial.print("Buffer length is ");
    Serial.println(bufferLength);
    Serial.print("Got command [");
    for (int i = 0; i < (bufferLength); i++) {
      Serial.print(cmdBuffer[i]);
    }
    Serial.println("]");*/
