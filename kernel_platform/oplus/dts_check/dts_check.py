#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import os
import csv
from operator import itemgetter, attrgetter
from argparse import ArgumentParser, FileType, ArgumentDefaultsHelpFormatter

class File:
    """struct to describe dtb file"""
    #attributes
    def __init__(self, d_totallines, file_name):
        self.file_name = file_name
        self.totallines = d_totallines
        self.rm_start_end_linenum = []
        self.rm_addr_size = []

    #function
    def get_reserved_tag_size(self):
        return len(self.rm_start_end_linenum)

    # get reg info for each tag area
    def get_reserved_memory_reglist(self):
        tmp_node = [-1] * 2

        for index in range(0, self.get_reserved_tag_size()):
            for line in self.totallines[self.rm_start_end_linenum[index][0]:self.rm_start_end_linenum[index][1]]:
                # find reg tag, get addr and size
                tag = line.find("reg")
                start = line.find("<")
                end = line.find(">")
                if tag != -1 and start != -1 and end != -1:
                    filter_tmp = filter(None, line[start + 1:end].split(" "))
                    num_list = list(filter_tmp)
                    tmp_node[0] = int(num_list[1], 16)
                    tmp_node[1] = int(num_list[3], 16)
                    self.rm_addr_size.append(tmp_node)
                    tmp_node = [-1] * 2

    # sort rm_addr_size with addr
    def sort_reserved_memory_reglist(self):
        self.rm_addr_size.sort(key=itemgetter(0))

    def print_reserved_memory_reglist(self):
        for index in self.rm_addr_size:
            print ("%s --- addr:%s, size:%s" % (self.file_name, hex(index[0]), hex(index[1])))

    # get start and end line of each tag
    def get_rm_start_end_line(self, tag):
        count = -999
        index_linenum = 1
        tmp_node = [-1] * 2

        for line in self.totallines:
            if line.find(tag) != -1:
                #print ("%s : get start line, start is %d" % (self.file_name, index_linenum))
                tmp_node[0] = index_linenum
                count = 0

            if count >= 0:
                # check tag until find match '}'
                left_brackets_exist = line.find('{')
                right_bracket_exist = line.find('}')
                if left_brackets_exist != -1:
                    count += 1
                if right_bracket_exist != -1:
                    count -= 1
                if count == 0:
                    #print ("%s : get end line, end is %d" % (self.file_name, index_linenum))
                    tmp_node[1] = index_linenum
                    self.rm_start_end_linenum.append(tmp_node)
                    # clear tmp variable
                    tmp_node = [-1] * 2
                    count = -999
            index_linenum += 1

class Dtbo_file(File):
    def get_linenum_of_tag(self):
        count = -999
        index_linenum = 1
        tag_list = []
        tmp_node = [-1] * 2     #tmp_node to save start and end of each tag

        for line in self.totallines:
            if line.find("reserved_memory =") != -1:
                # may have many fragment like this
                # reserved_memory = "/fragment@33:target:0", "/fragment@34:target:0";
                # need deal with each and get regs, all tag saved in tag_list
                filter_list = list(filter(None, line.split(" ")))
                for node in filter_list:
                    if node.find("fragment@") != -1:
                        # tag is like fragment@XX {
                        tag_list.append(node[node.find("fragment@"):node.find(":")] + " {")

                #print ("index_linenum is ", index_linenum, "tag_list is ", tag_list)
            index_linenum += 1

        for tag in tag_list:
            self.get_rm_start_end_line(tag)

def check_rm_addr_size_data(dtb, dtbo):
    test_addr = 0
    count = 0

    # sort
    data = dtb.rm_addr_size + dtbo.rm_addr_size
    data.sort(key=itemgetter(0))

    data_record_file = "%s_%s" %(dtb.file_name, dtbo.file_name)
    data_record_file = data_record_file.replace(".", "_")

    if sys.version_info > (3, 0):
        csv_file = open("reserved_layout.csv", 'w', newline='')
    else:
        csv_file = open("%s/%s.csv" %(sys.path[0], data_record_file), 'wb')

    title_name = [data_record_file, "addr", "addr_size"]
    writer = csv.DictWriter(csv_file, fieldnames=title_name)
    writer.writeheader()

    for index in data:
        count += 1
        if test_addr == 0:
            test_addr = index[0]
            test_addr += index[1]
            writer.writerow({data_record_file: count, "addr": hex(index[0]), "addr_size": hex(index[1])})
            #print ("test addr is %s, size is %s " % (hex(index[0]), hex(index[1])))
            #print ("---------------------------")
        else:
            if test_addr > index[0]:
                writer.writerow({data_record_file: count, "addr": hex(index[0]), "addr_size": hex(index[1])})
                print ("fatal error in (%s + %s) for addr %s is stepped!" %(dtb.file_name, dtbo.file_name, hex(index[0])))
                exit ("fatal error in (%s + %s) for addr %s is stepped!" %(dtb.file_name, dtbo.file_name, hex(index[0])))
            else:
                test_addr = index[0]
                test_addr += index[1]
                writer.writerow({data_record_file: count, "addr": hex(index[0]), "addr_size": hex(index[1])})
                #print ("test addr is %s, size is %s " % (hex(index[0]), hex(index[1])))
                #print ("---------------------------")
    csv_file.close()


def is_dtbo_file(file_name ,msm_arch):
    return file_name.find(msm_arch) != -1 and len(file_name) > 5 and file_name[-5:] == ".dtbo";

def main(**args):
    count = 0
    dtb_tag = "reserved-memory {"
    dtbo_class_list = []
    command_overlay_list = []
    dtbo_path_list = []
    tmp_list = []
    build_mode = args["build_mode"]
    msm_arch = args["msm_arch"]
    print ("args : build_mode is %s, msm_arch is %s" %(build_mode, msm_arch))

    # use dtc tool to get dts file
    # check_file_path is XXX/source/vnd/kernel_platform/oplus/dts_check
    # dtc_path is XXX/source/vnd/kernel_platform/oplus/dts_check
    # vnd path is XXX/source/vnd
    # dtc_path is XXX/source/vnd/prebuilts/misc/linux-x86/dtc/dtc
    check_file_path = sys.path[0]
    vnd_path = os.path.dirname(os.path.dirname(os.path.dirname(check_file_path)))
    if not os.path.exists(vnd_path):
        print ("vnd_path is not exist, exit!")
        #exit ("vnd_path is not exist, exit!")
        exit (0)

    f_dtb = open("%s/parameter.list" %(check_file_path), 'r')
    for line in f_dtb.readlines():
        if line[0] == '#':
            continue
        if line.find("dtc_tool_path") != -1:
            dtc_path = "%s/%s" %(vnd_path, line[line.find('<') + 1:line.find('>')])
        if line.find("dtb_file_path") != -1:
            dtb_path = "%s/%s" %(vnd_path, line[line.find("<") + 1:line.find('>')])
        if line.find("dtbo_file_path") != -1:
            tmp_list.append("%s/%s" %(vnd_path, line[line.find('<') + 1:line.find('>')]))

    '''
    if (not os.path.exists(dtc_path)) or (not os.path.exists(dtb_path)):
        print ("dtc_path or dtb_path is not exist, exit!")
        #exit ("dtc_path is not exist, exit!")
        exit (0)
    for index in tmp_list:
        if not os.path.exists(index):
            print ("%s is not exist, exit!" %(index))
            #exit ("%s is not exist, exit!" %(index))
            exit (0)
    '''

    if (build_mode == 'eng') or (build_mode == 'userdebug'):
        dtb_path = dtb_path.replace("gki", "consolidate")
        for index in tmp_list:
            dtbo_path_list.append(index.replace("gki", "consolidate"))
        tmp_list = []
    else:
        dtb_path = dtb_path.replace("consolidate", "gki")
        for index in tmp_list:
            dtbo_path_list.append(index.replace("consolidate", "gki"))
        tmp_list = []

    if not os.path.exists(dtb_path):
        print ("dtb_path is not exist, exit! dtb_path is %s" %(dtb_path))
        #exit ("dtb_path is not exist, exit!")
        exit (0)
    if len(dtbo_path_list) == 0:
        print ("dtbo_path is not exist, exit!")
        #exit ("dtbo_path is not exist, exit!")
        exit (0)

    dtb_result_name = "result_%s.dts" %(msm_arch)
    command_main = "%s -I dtb %s -q -O dts -o %s/%s" %(dtc_path, dtb_path, check_file_path, dtb_result_name)
    for path in dtbo_path_list:
        count += 1
        tmp_command = "%s -I dtb %s -q -O dts -o %s/result_dtbo_%s.dts" %(dtc_path, path, check_file_path, path[path.rfind("/")+1:path.rfind(".")])
        command_overlay_list.append(tmp_command)
        print command_main

    '''
    print ("command_main is : %s" %(command_main))
    for command in command_overlay_list:
        print ("command_dtbo is : %s" %(command))
    '''

    # use os.system to excute command and get result.dts and result_dtbo.dts
    status = os.system(command_main)
    if status != 0:
        print ("Decompile main dtb failed!")
        #exit ("Decompile main dtb failed!")
        exit (0)

    for command in command_overlay_list:
        status = os.system(command)
        if status != 0:
            print ("Decompile overlay dtb failed!")
            #exit ("Decompile overlay dtb failed!")
            exit (0)

    f_dtb = open("%s/%s" %(check_file_path, dtb_result_name), 'r')
    main_dtb_file = File(f_dtb.readlines(), dtb_result_name)
    main_dtb_file.get_rm_start_end_line(dtb_tag)
    main_dtb_file.get_reserved_memory_reglist()
    main_dtb_file.sort_reserved_memory_reglist()
    #main_dtb_file.print_reserved_memory_reglist()

    result_file_list = os.listdir(check_file_path)
    for file in result_file_list:
        if file.find("result_dtbo") != -1 and file[-4:] == ".dts":
            #print ("file is %s" %(file))
            f_dtbo = open("%s/%s" %(check_file_path, file), 'r')
            overlay_dtbo_file = Dtbo_file(f_dtbo.readlines(), file)
            overlay_dtbo_file.get_linenum_of_tag()
            overlay_dtbo_file.get_reserved_memory_reglist()
            overlay_dtbo_file.sort_reserved_memory_reglist()
            dtbo_class_list.append(overlay_dtbo_file)

    for file in dtbo_class_list:
        file.print_reserved_memory_reglist()
        result = check_rm_addr_size_data(main_dtb_file, file)

    f_dtb.close()
    f_dtbo.close()


if __name__ == '__main__':
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument("-m","--build-mode", dest="build_mode", help="specify the build mode to build kernel.", default="")
    parser.add_argument("-a","--msm-arch", dest="msm_arch", help="specify the msm_arch type to build kernel.", default="")
    args = parser.parse_args()
    main(**args.__dict__)