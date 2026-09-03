#pragma once

#include "ShowStreamReader.h"

#include <AP_Filesystem/AP_Filesystem.h>
#include <fcntl.h>

// storage location of the show file (SITL: relative to the working dir)
#ifndef HAL_BOARD_SHOW_DIRECTORY
#  if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#    define HAL_BOARD_SHOW_DIRECTORY "./show"
#  else
#    define HAL_BOARD_SHOW_DIRECTORY "/SHOW"
#  endif
#endif
#define SHOW_FILE (HAL_BOARD_SHOW_DIRECTORY "/show.bin")

/*
  ShowFileSource - real filesystem BlockSource for ShowStreamReader.
  Sequential reads over an open file descriptor via AP::FS.
*/
class ShowFileSource : public ShowStreamReader::BlockSource {
public:
    bool open() override
    {
        _fd = AP::FS().open(SHOW_FILE, O_RDONLY);
        return _fd >= 0;
    }
    int32_t read(void *buf, uint32_t len) override
    {
        return AP::FS().read(_fd, buf, len);
    }
    bool rewind() override
    {
        return AP::FS().lseek(_fd, 0, SEEK_SET) == 0;
    }
    void close() override
    {
        if (_fd >= 0) {
            AP::FS().close(_fd);
            _fd = -1;
        }
    }
private:
    int _fd = -1;
};
