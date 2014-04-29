#include"logfile.h"

LogFile::LogFile()
{
	_stream = NULL;
	_Mutex = NULL;
	_F_timestamp = 1;
	_F_counterstamp = 1;
	_counter = 0;
	_dir = "/root/";
}

LogFile::~LogFile()
{
	if (_Mutex)
		pthread_mutex_lock(_Mutex);

	if (_stream) {
		fflush(_stream);
		fclose(_stream);
		_stream = NULL;
	}

	if (_Mutex)
		pthread_mutex_unlock(_Mutex);
}

int LogFile::setDir(const char *dir_path)
{
	if (_stream)
		return -1;

	string path = dir_path;
	int ret = _testDirPath(path);
	if (ret == 0)
		_dir = dir_path;
	else
		_dir = "/root/";

	return ret;
}

int LogFile::setDir(string dir_path)
{
	if (_stream)
		return -1;

	int ret = _testDirPath(dir_path);
	if (ret == 0)
		_dir = dir_path;
	else
		_dir = "/root/";

	return ret;
}

int LogFile::setDefaultDir()
{
	if (_stream)
		return -1;

	_dir = "/root/";
	return 0;
}
    
int LogFile::setMutex(pthread_mutex_t *mutex)
{
	if (_stream) 
		return -1;

	if (!_Mutex) {
		_Mutex = mutex;
		return 0;
	} else {
		return -2;
	}
}

int LogFile::unsetMutex()
{
	if (_stream)
		return -1;
	if (_Mutex) {
		pthread_mutex_lock(_Mutex);
		pthread_mutex_t *temp = _Mutex;
		_Mutex = NULL;
		pthread_mutex_unlock(temp);
		return 0;
	} else {
		return -2;
	}
}
    
int LogFile::timeStampON()
{
	if (_stream)
		return -1;

	_F_timestamp = 1;
	return 0;
}

int LogFile::timeStampOFF()
{
	if (_stream)
		return -1;
	_F_timestamp = 0;
	return 0;
}

int LogFile::counterStampON()
{
	if (_stream)
		return -1;
	_F_counterstamp = 1;
	_counter = 1;
	return 0;
}

int LogFile::counterStampOFF()
{
	if (_stream)
		return -1;
	_F_counterstamp = 0;
	_counter = 0;
	return 0;
}
    
int LogFile::open()
{
	if (_stream)
		return -1;

	int ret = _testDirPath(_dir);
	if (ret)
		return ret;

	ret = _findFileName();
	if (ret)
		return ret;

	_stream = fopen(_file_path.c_str(), "w");
	if (!_stream)
		return -2;

	return 0;
}

int LogFile::open(const char *path)
{
	if (_stream)
		return -1;
	
	_stream = fopen(path, "a");
	if (!_stream)
		return -2;

	return 0;
}
	
int LogFile::open(string path)
{
	if (_stream)
		return -1;

	_stream = fopen(path.c_str(), "a");
	if (!_stream) 
		return -2;

	return 0;
}
    
bool LogFile::isOpen()
{
	if (_stream)
		return true;
	else
		return false;
}

int LogFile::write(const char *fmt, ...)
{
	if (!_stream) 
		return -1;

	if (_Mutex) 
		pthread_mutex_lock(_Mutex);

	va_list args;
	va_start(args, fmt);
	if (_F_counterstamp)
		_writeCount();
	if (_F_timestamp)
		_writeTime();

	int count = vfprintf(_stream, fmt, args);
	va_end(args);
	
	if (_Mutex) {
		pthread_mutex_unlock(_Mutex);
	}

	return count;
}

int LogFile::writeErr(const char *str, int errnum)
{
	if (!_stream)
		return -1;
	if (_Mutex)
		pthread_mutex_lock(_Mutex);
	if (_F_counterstamp)
		_writeCount();
	if (_F_timestamp)
		_writeTime();
	char *errstr = strerror(errnum);
	int count = fprintf(_stream, "%s : %s\n", str, errstr);
	if (_Mutex)
		pthread_mutex_unlock(_Mutex);
	return count;
}
    
int LogFile::flush()
{
	if (!_stream)
		return -1;
	if (_Mutex)
		pthread_mutex_lock(_Mutex);

	int ret = fflush(_stream);

	if (_Mutex)
		pthread_mutex_unlock(_Mutex);

	return ret;
}

int LogFile::close()
{
	if (!_stream) {
		return -1;
	}

	if (_Mutex)
		pthread_mutex_lock(_Mutex);

	int ret = fclose(_stream);
	_stream = NULL;

	if (_Mutex)
		pthread_mutex_unlock(_Mutex);

	return ret;
}

int LogFile::_findFileName()
{
	struct tm time_b;
	time_t t = time(NULL);
	localtime_r(&t, &time_b);
	char buffer[PATH_MAX];
	memset(buffer, 0, PATH_MAX);

	int i, fd;
	for (i=1; i<1000 ; i++) {
		sprintf(buffer, "%s%02d%02d%02d_%03d", _dir.c_str(),
			time_b.tm_year+1900, time_b.tm_mon+1, time_b.tm_mday, i);

		fd = ::open(buffer, O_CREAT | O_EXCL, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
		if (fd < 0) {
			if (errno == EEXIST)
				continue;
			else 
				return -1;
		}
		else {
			::close(fd);
			_file_path= buffer;
			return 0;
		}
	}
	return 0;
}

int LogFile::_testDirPath(string dir_path)
{
	/* 
	   Return value :
	    0 : Directory exist
	   -1 : Not a Directory
	   -2 : Directory not exist
	   -3 : Others
	*/
	DIR *dir = opendir(dir_path.c_str());
	if (!dir) {
		if (errno == ENOTDIR)
			return -1;
		else if (errno == ENOENT)
			return -2;
		else
			return -3;
	}

	int l = dir_path.size();
	if (dir_path.at(l-1) != '/')
		dir_path += '/';
	
	_dir = dir_path;
	closedir(dir);
	return 0;
}

int LogFile::_writeTime()
{
	struct tm time_b;
	time_t t = time(NULL);
	localtime_r(&t, &time_b);
	int ret = fprintf(_stream, "[%04d-%02d-%02d %02d:%02d:%02d] : ", 
			time_b.tm_year+1900, 
			time_b.tm_mon+1, 
			time_b.tm_mday,
			time_b.tm_hour,
			time_b.tm_min,
			time_b.tm_sec);

	return ret;
}

int LogFile::_writeCount()
{
	int ret = fprintf(_stream, "%03d ", _counter);
	_counter++;
	return ret;
}
