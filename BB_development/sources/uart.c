#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>

struct termios *configure;
void tty_config(struct termios *con, int descriptor);
char *device = "/dev/ttyO2";
int fd;

void tty_config(struct termios *con, int descriptor)
{
    tcgetattr(descriptor, con);
    con->c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    con->c_oflag = 0;
    con->c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    con->c_cc[VMIN] = 1;
    con->c_cc[VTIME] = 0;
    if(cfsetispeed(con, B57600) || cfsetospeed(con, B57600))
    {
        perror("ERROR in baud set\n");
    }

    if(tcsetattr(descriptor, TCSAFLUSH, con) < 0)
    {
        perror("ERROR in set attr\n");
    }
}

void uart_init(void)
{
    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd == -1)
    {
        perror("ERROR opening file descriptor\n");
    }

    configure = (struct termios*)malloc(sizeof(struct termios));
    tty_config(configure, fd);
}
