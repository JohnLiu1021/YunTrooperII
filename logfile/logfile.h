#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<dirent.h>
#include<sys/types.h>
#include<time.h>
#include<pthread.h>
#include<limits.h>
#include<stdarg.h>
#include<string.h>
#include<errno.h>
#include<string>

using namespace std;
class LogFile {
public:
	LogFile();
	~LogFile();

	int setDir(const char *);
	int setDir(string);
	int setDefaultDir();

	int setMutex(pthread_mutex_t *);
	int unsetMutex();

	int timeStampON();
	int timeStampOFF();
	int counterStampON();
	int counterStampOFF();

	int open();
	int open(const char *);
	int open(string);

	int write(const char *, ...);
	int writeErr(const char *, int errnum);

	int flush();
	int close();

private:
	FILE *_stream;
	pthread_mutex_t *_Mutex;
	unsigned char _F_timestamp;
	unsigned char _F_counterstamp;
	string _dir;
	string _file_path;
	int _counter;

	int _findFileName();
	int _testDirPath(string);
	int _writeTime();
	int _writeCount();
};

