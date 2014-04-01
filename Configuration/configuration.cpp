#include "configuration.h"

int Configuration::readFromFile(const char *path)
{
	std::ifstream fin(path);
	if (!fin) {
		return -1;
	}

	std::string name;
	double value_double;

	while(fin >> name >> value_double) {
		for(it=options.begin(); it!=options.end(); it++) {
			if (name == it->name) {
				it->value = value_double;
			}
		}
	}
	if (!fin.eof())
		return -1;
	
	fin.close();
	return 0;
}

int Configuration::writeToFile(const char *path)
{
	std::ofstream fout(path);
	if (!fout) {
		return -1;
	}
	
	for (it=options.begin(); it!=options.end(); it++)
		fout << it->name << " " << it->value << std::endl;

	fout.close();
	return 0;
}

bool Configuration::search(const char *name, double *value)
{
	for (it=options.begin(); it!=options.end(); it++) {
		if (it->name == name) {
			*value = it->value;
			return true;
		}
	}
	return false;
}

bool Configuration::search(const char *name, double &value)
{
	for (it=options.begin(); it!=options.end(); it++) {
		if (it->name == name) {
			value = it->value;
			return true;
		}
	}
	return false;
}
