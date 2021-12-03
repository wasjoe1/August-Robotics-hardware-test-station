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
 * @file   ncurses_utils.h
 * @author Roy Zhang (roy.zhang@augustrobotics.com)
 * @date   25-10-2021
 *
 * @brief
 *
 */
#ifndef NCURSES_UTILS_H
#define NCURSES_UTILS_H
#include <ncurses.h>
#include <iostream>
#include <string>
#include <vector>
namespace august
{
class NcursesUtils
{
public:
    /**
     * @brief  Construct a new NcursesUtils.
     *
     */
    NcursesUtils();

    /**
     * @brief  Destroy the NcursesUtils.
     *
     */
    ~NcursesUtils();

    /**
     * @brief  Add new header column
     *
     * @param  size : Width in front of the coloumn
     * @param  title : Text of the title
     */
    void addTitle(int size, const std::string& title);

    /**
     * @brief
     *
     */
    void clearTitle();

    /**
     * @brief  Get width size of the idx-th title field
     *
     * @param  idx : Index of the coloumn
     * @return int : Width in front of idx-th the coloumn
     */
    int getSize(int idx) const;
    /**
     * @brief  Get Title text
     *
     * @param  idx : index of the column
     * @return std::string : The header
     */
    std::string getTitle(int idx) const;

    /**
     * @brief  Get number of titles in the class
     *
     * @return size_t : Numbe of headers
     */
    size_t getTotalSize() const;

    /**
     * @brief  Ncurse print header row
     *
     * @param  row : Row in the screen
     */
    void paintHeaderRow(int row) const;

    /**
     * @brief
     *
     * @param  window :
     * @param  row :
     */
    void paintWindowHeaderRow(WINDOW* window, int row) const;

    /**
     * @brief  Paint line, width of the screen
     *
     * @param  row : Row of ther line
     * @param  character : char used for the line. default='-'
     */
    void paintFullRow(int row, char character = '-');

    /**
     * @brief  If condition print color
     * Print text in color if condition is true
     *
     * @param  row : Row
     * @param  col : Column
     * @param  format : Format i.e "%s"
     * @param  str : Text to print
     * @param  if_condition : If-condition
     * @param  color_num : Number of the color
     */
    void ifCPrint(
        int row, int col, const std::string& format, const std::string& str, int color_num, bool if_condition = true);

    void ifCPrint(int row, int col, const std::string& format, double str, int color_num, bool if_condition = true);

    void ifCPrint(int row, int col, const std::string& format, int str, int color_num, bool if_condition = true);
    /**
     * @brief
     *
     * @param  row :
     * @param  col :
     * @param  format :
     * @param  str :
     * @param  color_num :
     * @param  if_condition :
     */
    void ifWindowColorPrint(WINDOW* sub_window,
                            int row,
                            int col,
                            const std::string& format,
                            const std::string& str,
                            int color_num,
                            bool if_condition = true);

    void ifWindowColorPrint(WINDOW* sub_window,
                            int row,
                            int col,
                            const std::string& format,
                            double str,
                            int color_num,
                            bool if_condition = true);

    void ifWindowColorPrint(WINDOW* sub_windown,
                            int row,
                            int col,
                            const std::string& format,
                            int str,
                            int color_num,
                            bool if_condition = true);

private:
    std::vector<std::string> title_;
    std::vector<int> size_;
};

}  // namespace august

#endif
