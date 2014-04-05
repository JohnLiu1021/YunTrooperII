#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <syslog.h>
#include <unistd.h>

#define PREREQUISITE_FILE "/root/prerequisite"
#define EXEC_PROGRAM "/root/Navigation"
#define TRIED_TIME 10

int main(void)
{
	/* Daemon function */
	if (daemon(0, 0) != 0) {
		syslog(LOG_ERR, "startd: Daemon Error: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}

	syslog(LOG_NOTICE, "startd: daemon is running!\n");

	if (chdir("/root") == -1) {
		syslog(LOG_ERR, "startd: change working directory: %s\n", strerror(errno));
	}

	/* Wait for 2 second to start */
	sleep(2);

	/* Prerequisite file */
	char stringBuffer[255];
	FILE *file;
	file = fopen(PREREQUISITE_FILE, "r");
	if (file == NULL) {
		syslog(LOG_ERR, "startd: fopen prerequisite file: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}

	/* Testing each file listed in prerequisite file */
	while (fgets(stringBuffer, 255, file) != NULL) {
		stringBuffer[strlen(stringBuffer) - 1] = '\0';	// Remove the new line character.
		int triedCounter;
		for (triedCounter = 0; triedCounter < TRIED_TIME; triedCounter++) {
			int fd = open(stringBuffer, O_RDWR);
			if (fd <= 0) {
				if (errno == ENOENT) {
					syslog(LOG_INFO, "startd: File %s is not ready.\n", stringBuffer);
				} else {
					syslog(LOG_ERR, "startd: open %s: %s\n", stringBuffer, strerror(errno));
					exit(EXIT_FAILURE);
				}
			} else {
				syslog(LOG_NOTICE, "startd: File %s is ready.\n", stringBuffer);
				break;
			}
			sleep(1);
		}

		if (triedCounter >= TRIED_TIME) {
			syslog(LOG_ERR, "startd: Could not open file %s.\n", stringBuffer);
			fclose(file);
			exit(EXIT_FAILURE);
		}
	} 

	/* All requisite are satified, exec the navigation file */
	if (feof(file)) {
		syslog(LOG_NOTICE, "startd: All prerequisites are satified.\n");
		syslog(LOG_NOTICE, "startd: Starting navigation program.");
		fclose(file);
		int ret = execl(EXEC_PROGRAM, EXEC_PROGRAM, NULL);
		if (ret) {
			syslog(LOG_ERR, "startd: Failed to start navigation program: %s\n", strerror(errno));
			exit(EXIT_FAILURE);
		}

	/* Error occurred! */
	} else {
		syslog(LOG_ERR, "startd: fgets: %s\n", strerror(errno));
		fclose(file);
		exit(EXIT_FAILURE);
	}
			
	return 0;
}
