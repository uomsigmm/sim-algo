#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFFER_SIZE 32

int getInteger(char *command) {
  printf("%s\n", command);
  fflush(stdout);
  char response[BUFFER_SIZE];
  if (fgets(response, BUFFER_SIZE, stdin) == NULL) {
    fprintf(stderr, "ERROR: fgets failed in getInteger for '%s'\n", command);
    exit(1);
  }
  int value = atoi(response);
  return value;
}

int getBoolean(char *command) {
  printf("%s\n", command);
  fflush(stdout);
  char response[BUFFER_SIZE];
  if (fgets(response, BUFFER_SIZE, stdin) == NULL) {
    fprintf(stderr, "ERROR: fgets failed in getBoolean for '%s'\n", command);
    exit(1);
  }
  int value = (strcmp(response, "true\n") == 0);
  return value;
}

int getAck(char *command) {
  printf("%s\n", command);
  fflush(stdout);
  char response[BUFFER_SIZE];
  if (fgets(response, BUFFER_SIZE, stdin) == NULL) {
    fprintf(stderr, "ERROR: fgets failed in getAck for '%s'\n", command);
    exit(1);
  }
  int success = (strcmp(response, "ack\n") == 0);
  return success;
}

int API_mazeWidth(void) {
  return getInteger("mazeWidth");
}

int API_mazeHeight(void) {
  return getInteger("mazeHeight");
}

int API_wallFront(void) {
  return getBoolean("wallFront");
}

int API_wallRight(void) {
  return getBoolean("wallRight");
}

int API_wallLeft(void) {
  return getBoolean("wallLeft");
}

int API_moveForward(void) {
  return getAck("moveForward");
}

void API_turnRight(void) {
  getAck("turnRight");
}

void API_turnLeft(void) {
  getAck("turnLeft");
}

void API_turnRight45(void) {
  getAck("turnRight45");
}

void API_turnLeft45(void) {
  getAck("turnLeft45");
}

void API_setWall(int x, int y, char direction) {
  printf("setWall %d %d %c\n", x, y, direction);
  fflush(stdout);
}

void API_clearWall(int x, int y, char direction) {
  printf("clearWall %d %d %c\n", x, y, direction);
  fflush(stdout);
}

void API_setColor(int x, int y, char color) {
  printf("setColor %d %d %c\n", x, y, color);
  fflush(stdout);
}

void API_clearColor(int x, int y) {
  printf("clearColor %d %d\n", x, y);
  fflush(stdout);
}

void API_clearAllColor(void) {
  printf("clearAllColor\n");
  fflush(stdout);
}

void API_setText(int x, int y, char *text) {
  printf("setText %d %d %s\n", x, y, text);
  fflush(stdout);
}

void API_clearText(int x, int y) {
  printf("clearText %d %d\n", x, y);
  fflush(stdout);
}

void API_clearAllText(void) {
  printf("clearAllText\n");
  fflush(stdout);
}

int API_wasReset(void) {
  return getBoolean("wasReset");
}

void API_ackReset(void) {
  getAck("ackReset");
}
