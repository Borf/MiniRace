#include "MiniRace.h"
#include <blib/util/FileSystem.h>

#pragma comment(lib,"blib.lib")

int main(int argc, char* argv[])
{
	blib::util::FileSystem::registerHandler(new blib::util::PhysicalFileSystemHandler());
	blib::util::FileSystem::registerHandler(new blib::util::PhysicalFileSystemHandler(".."));
	blib::util::FileSystem::registerHandler(new blib::util::PhysicalFileSystemHandler("../blib"));

	MiniRace race;
	race.start();
	return 0;
}