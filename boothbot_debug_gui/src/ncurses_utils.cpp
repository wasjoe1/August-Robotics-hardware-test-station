/**
 * ________________________________________________________________________
 * August Robotics    CONFIDENTIAL
 * ________________________________________________________________________
 *
 * This file is part of August Robotics
 *
 * Copyright (c) 2021 August Robotics,
 * All Rights Reserved.
 *
 * Unauthorized copying of this file and software, partly or in whole, via any
 * medium is strictly prohibited, including all derived formats such as object
 * code or binaries.
 *
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of August Robotics. The intellectual and technical concepts
 * contained herein are proprietary to August Robotics. and may be covered by
 * U.S and Foreign Patents, patents in process, and are protected by
 * trade secret or copyright law. Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior
 * written permission is obtained from August Robotics.
 *
 * @file   ncurses_utils.cpp
 * @author Roy Zhang (roy.zhang@augustrobotics.com)
 * @date   25-10-2021
 *
 * @brief
 *
 */
#include <stdarg.h>  // va_start, etc.
#include <memory>    // std::unique_ptr
#include <cstring>   // strcpy
#include "ros_boothbot_debug_gui/ncurses_utils.h"
namespace august
{
NcursesUtils::NcursesUtils()
{
}
NcursesUtils::~NcursesUtils()
{
}
void NcursesUtils::addTitle(int size, const std::string& title)
{
    title_.push_back(title);
    size_.push_back(size);
}
void NcursesUtils::clearTitle()
{
    title_.clear();
    size_.clear();
}
int NcursesUtils::getSize(int idx) const
{
    return size_.at(idx);
}

std::string NcursesUtils::getTitle(int idx) const
{
    return title_.at(idx);
}

size_t NcursesUtils::getTotalSize() const
{
    return title_.size();
}

void NcursesUtils::paintHeaderRow(int row) const
{
    int col = 0;
    for (size_t i = 0; i < getTotalSize(); i++)
    {
        mvprintw(row, col += getSize(i), "%s", getTitle(i).c_str());
    }
}
void NcursesUtils::paintWindowHeaderRow(WINDOW* window, int row) const
{
    int col = 0;
    for (size_t i = 0; i < getTotalSize(); i++)
    {
        mvwprintw(window, row, col += getSize(i), getTitle(i).c_str());
    }
    wrefresh(window);
}
void NcursesUtils::paintFullRow(int row, char character)
{
    int width = 0, height = 0;
    getmaxyx(stdscr, height, width);

    std::string line;
    for (int i = 0; i < width; i++)
    {
        line.push_back(character);
    }
    mvprintw(row, 0, "%s", line.c_str());
}

void NcursesUtils::ifCPrint(
    int row, int col, const std::string& format, const std::string& str, int color_num, bool if_condition)
{
    if (if_condition)
    {
        attron(COLOR_PAIR(color_num));
    }
    mvprintw(row, col, format.c_str(), str.c_str());
    if (if_condition)
    {
        attroff(COLOR_PAIR(color_num));
    }
}
void NcursesUtils::ifCPrint(int row, int col, const std::string& format, double str, int color_num, bool if_condition)
{
    if (if_condition)
    {
        attron(COLOR_PAIR(color_num));
    }
    mvprintw(row, col, format.c_str(), str);
    if (if_condition)
    {
        attroff(COLOR_PAIR(color_num));
    }
}

void NcursesUtils::ifCPrint(int row, int col, const std::string& format, int str, int color_num, bool if_condition)
{
    if (if_condition)
    {
        attron(COLOR_PAIR(color_num));
    }
    mvprintw(row, col, format.c_str(), str);
    if (if_condition)
    {
        attroff(COLOR_PAIR(color_num));
    }
}

void NcursesUtils::ifWindowColorPrint(WINDOW* sub_window,
                                      int row,
                                      int col,
                                      const std::string& format,
                                      const std::string& str,
                                      int color_num,
                                      bool if_condition)
{
    wmove(sub_window, row, col);
    wclrtoeol(sub_window);
    if (if_condition)
    {
        wattron(sub_window, COLOR_PAIR(color_num));
    }
    mvwprintw(sub_window, row, col, format.c_str(), str.c_str());
    if (if_condition)
    {
        wattroff(sub_window, COLOR_PAIR(color_num));
    }
    wrefresh(sub_window);
}
void NcursesUtils::ifWindowColorPrint(
    WINDOW* sub_window, int row, int col, const std::string& format, double str, int color_num, bool if_condition)
{
    wmove(sub_window, row, col);
    wclrtoeol(sub_window);
    if (if_condition)
    {
        wattron(sub_window, COLOR_PAIR(color_num));
    }
    mvwprintw(sub_window, row, col, format.c_str(), str);
    if (if_condition)
    {
        wattroff(sub_window, COLOR_PAIR(color_num));
    }
    wrefresh(sub_window);
}

void NcursesUtils::ifWindowColorPrint(
    WINDOW* sub_window, int row, int col, const std::string& format, int str, int color_num, bool if_condition)
{
    wmove(sub_window, row, col);
    wclrtoeol(sub_window);
    if (if_condition)
    {
        wattron(sub_window, COLOR_PAIR(color_num));
    }
    mvwprintw(sub_window, row, col, format.c_str(), str);
    if (if_condition)
    {
        wattroff(sub_window, COLOR_PAIR(color_num));
    }
    wrefresh(sub_window);
}
}  // namespace august
