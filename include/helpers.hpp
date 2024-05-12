#include <stdio.h>
#include <stdbool.h>
#include <memory>
#include <string>
#include <ctime>

bool FileExists(const char *filename)
{
    FILE *fp = fopen(filename, "r");
    bool is_exist = false;
    if (fp != NULL)
    {
        is_exist = true;
        fclose(fp); // close the file
    }
    return is_exist;
}

std::string GetCurrentTime()
{
    tm *currentLocalTime;
    time_t currentTime;
    char dateChar[100];
    char timeChar[100];
    time(&currentTime);
    currentLocalTime = localtime(&currentTime);

    strftime(dateChar, 50, "%d%m%y", currentLocalTime);
    strftime(timeChar, 50, "%H%M%S", currentLocalTime);

    std::string dateString(dateChar);
    std::string timeString(timeChar);

    std::string currentTimeString = dateString + "_" + timeString;

    return currentTimeString;
}
