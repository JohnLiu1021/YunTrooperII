#ifndef CONFIGURATION_H
#define CONFIGURARION_H 1

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class Configuration {
public:
	int readFromFile(const char *);
	int writeToFile(const char *);
	bool search(const char *, double *);
	bool search(const char *, double &);

	struct Option {
		std::string name;
		double value;
	};

	std::vector<struct Option> options;
	std::vector<struct Option>::iterator it;
};

#endif
