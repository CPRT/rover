#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <strings.h>
#include <sys/socket.h>

#include "msgdefs.h"
#include "packet.h"

#define NAME_LENGTH 128

pthread_mutex_t lock;
DriveCtrlCmd msg;
uint16_t seq = 0;
struct sockaddr_in servaddr;

void *reader(void *arg) {
  uint8_t buf[sizeof(msg) + 6];
  int sockfd;

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    printf("\n Error : Connect Failed \n");
    exit(0);
  }

  while (1) {
    pthread_mutex_lock(&lock);

    SpacePacket::encode(buf, &msg, sizeof(msg), PacketType::COMMAND,
                        APID_DRIVECTRLCMD, seq++);
    sendto(sockfd, buf, sizeof(buf), 0, (struct sockaddr *)NULL,
           sizeof(servaddr));

    pthread_mutex_unlock(&lock);
    usleep(4000);
  }

  close(sockfd);
  return NULL;
}

int main(int argc, char **argv) {
  int fd;
  unsigned char axes = 2;
  unsigned char buttons = 2;
  int version = 0x000800;

  if (argc != 3) {
    printf("%s: <remoteip> <joystick>\n", argv[0]);
    return 1;
  }

  if ((fd = open(argv[argc - 1], O_RDONLY)) < 0) {
    perror("jstest");
    return 1;
  }

  ioctl(fd, JSIOCGVERSION, &version);
  ioctl(fd, JSIOCGAXES, &axes);
  ioctl(fd, JSIOCGBUTTONS, &buttons);

  int *axis;
  char *button;
  struct js_event js;

  axis = (int *)calloc(axes, sizeof(int));
  button = (char *)calloc(buttons, sizeof(char));

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_addr.s_addr = inet_addr(argv[1]);
  servaddr.sin_port = htons(1235);
  servaddr.sin_family = AF_INET;

  if (pthread_mutex_init(&lock, NULL) != 0) {
    printf("Mutex initialization failed.\n");
    return 1;
  }

  pthread_t reader_thread;
  pthread_create(&reader_thread, NULL, reader, NULL);

  while (1) {
    if (read(fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
      perror("\njstest: error reading");
      return 1;
    }

    switch (js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_BUTTON:
      button[js.number] = js.value;
      break;
    case JS_EVENT_AXIS:
      axis[js.number] = js.value;
      break;
    }

    pthread_mutex_lock(&lock);
    msg.x = axis[1];
    msg.y = axis[0];
    msg.yaw = axis[2];
    pthread_mutex_unlock(&lock);

    printf("\r");
    printf("Channels:");
    printf("  x:%5" PRIu16, msg.x);
    printf("  y:%5" PRIu16, msg.y);
    printf("yaw:%5" PRIu16, msg.yaw);
    fflush(stdout);
  }

  pthread_join(reader_thread, NULL);

  close(fd);

  return -1;
}