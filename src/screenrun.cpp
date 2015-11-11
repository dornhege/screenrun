#include <ros/ros.h>
#include <sys/select.h>
#include <stdio.h>
#include <dirent.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <sstream>
using namespace std;

enum MODE
{
    MODE_SCREEN,
    MODE_X,
};

enum MODE g_mode = MODE_SCREEN;
string g_screen_cmd = "screen";

struct Params
{
    std::string keyNewTab;
    double sleepNewTab;
    std::string keyTabTitle;
    double sleepTabTitle;
    double sleepCommand;

    bool load() {
        ros::NodeHandle nhPriv("~");
        nhPriv.param("key_new_tab", keyNewTab, std::string("ctrl+shift+t"));
        nhPriv.param("sleep_new_tab", sleepNewTab, 2.0);
        nhPriv.param("key_tab_title", keyTabTitle, std::string(""));
        nhPriv.param("sleep_tab_title", sleepTabTitle, 1.0);
        nhPriv.param("sleep_command", sleepCommand, 1.0);
        if(keyNewTab.empty()) {
            ROS_FATAL("~key_new_tab param was empty.");
            return false;
        }
        return true;
    }
};
Params g_params;

bool executeCmd(const string & cmd)
{
    int ret = system(cmd.c_str());
    if(ret != 0) {
        perror("executeCmd");
        ROS_ERROR("Command \"%s\" returned %d", cmd.c_str(), ret);
        return false;
    }
    return true;
}

bool queryCmd(const string & cmd, string & result)
{
    FILE* p = popen(cmd.c_str(), "r");
    if(!p)
        return false;
    // read pipe
    char buf[1024];
    char* rr = fgets(buf, 1023, p);
    if(rr == NULL) {
        ROS_ERROR("Comamnd \"%s\" read from pipe failed.", cmd.c_str());
        return false;
    }
    result = buf;
    if(!result.empty()) {
        if(result[result.size() - 1] == '\n') {
            result = result.substr(0, result.size() - 1);
        }
    }

    int ret = pclose(p);
    if(ret != 0) {
        perror("queryCmd");
        ROS_ERROR("Command \"%s\" returned %d", cmd.c_str(), ret);
        return false;
    }
    return true;
}

class ProgramEntry
{
    public:
        string name;
        vector<string> commands;

        void pushToScreen() {
            ROS_INFO("Creating screen window for \"%s\"", name.c_str());
            if(!executeCmd(g_screen_cmd + " -S ros -X screen -t '" + name + "'"))
                return;
            for(vector<string>::iterator it = commands.begin(); it != commands.end(); it++) {
                ROS_INFO("Pushing command: \"%s\"", it->c_str());
                if(!executeCmd(g_screen_cmd + " -p '" + name + "' -S ros -X eval 'stuff \"" + *it + "\"'"))
                    return;
            }
        }

        /// Commands for screen might be 'do_stuff\015', where screen interprets the \015 as return.
        /// For X Windows, we need this information explicitly.
        /** 
         *  This functions removes a \015 appendix, if it was there and returns true in that case,
         *  false otherwise.
         */
        bool filterCmdReturn(std::string & cmd) {
            if(cmd.length() < 4)
                return false;
            if(cmd.substr(cmd.length() - 4) == "\\015") {
                cmd = cmd.substr(0, cmd.length() - 4);
                return true;
            }
            return false;
        }

        std::string toString(double d) {
            std::stringstream ss;
            ss << d;
            return ss.str();
        }

        void pushToXWindow(const std::string & wid) {
            ROS_INFO("Creating window/tab for \"%s\"", name.c_str());
            string cmd = "xdotool ";
            if(!executeCmd(cmd + "windowfocus " + wid))
                return;
            // new tab
            if(!executeCmd(cmd + "key --clearmodifiers " + g_params.keyNewTab))
                return;
            if(!executeCmd(cmd + "sleep " + toString(g_params.sleepNewTab)))
                return;
            // set tab title
            if(!g_params.keyTabTitle.empty()) {
                if(!executeCmd(cmd + "key " + g_params.keyTabTitle))
                    return;
                if(!executeCmd(cmd + "type " + name))
                    return;
                if(!executeCmd(cmd + "key Return"))
                    return;
                if(!executeCmd(cmd + "sleep " + toString(g_params.sleepTabTitle)))
                    return;
            }
            // enter commands into tab
            for(vector<string>::iterator it = commands.begin(); it != commands.end(); it++) {
                std::string pushCmd = *it;
                bool doReturn = filterCmdReturn(pushCmd);
                ROS_INFO("Pushing command: \"%s\"", pushCmd.c_str());
                if(!executeCmd(cmd + "type --delay 1 --clearmodifiers \"" + pushCmd.c_str() + "\""))
                    return;
                if(doReturn)
                    executeCmd(cmd + "key Return"); // this would be OK to fail.
                if(!executeCmd(cmd + "sleep " + toString(g_params.sleepCommand)))
                    return;
            }
            // switch window active
            cmd = "wmctrl ";
            if(!executeCmd(cmd + "-i -a " + wid))
                return;
        }
};

vector<ProgramEntry> programs;

bool load()
{
    ros::NodeHandle nh("~");

    XmlRpc::XmlRpcValue xmlRpc;
    if(!nh.getParam("programs", xmlRpc)) {
        ROS_FATAL("No programs defined.");
        return false;
    } 

    if(xmlRpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_FATAL("programs param should be a list.");
        return false;
    }
    if(xmlRpc.size() == 0) {
        ROS_FATAL("programs list is empty.");
        return false;
    }

    for(int i = 0; i < xmlRpc.size(); i++) {
        ProgramEntry pe;

        if(xmlRpc[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_FATAL("programs entry %d is not of type string.", i);
            return false;
        }
        pe.name = (string)xmlRpc[i]["name"];
        XmlRpc::XmlRpcValue cmds = xmlRpc[i]["commands"];
        if(cmds.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_FATAL("commands for %s is not a list", pe.name.c_str());
            return false;
        }
        for(int j = 0; j < cmds.size(); j++) {
            pe.commands.push_back((string)cmds[j]);
        }

        programs.push_back(pe);
    }

    return true;
}

string getScreenPath()
{
    char* user = getenv("USER");
    if(!user)
        return "";

    return string("/var/run/screen/S-") + user;
}

bool screenRunning()
{
    string sp = getScreenPath();
    if(sp.empty())
        return false;

    DIR* dir = opendir(sp.c_str());
    if(!dir)
        return false;
    struct dirent* entry;
    while( (entry = readdir(dir)) ) {
        string dname = entry->d_name;
        if(dname.find("ros") != string::npos)
            return true;
    }
    return false;
}

bool command_exists(const std::string & cmd)
{
    std::string query = std::string("which ") + cmd + " > /dev/null";
    int ret = system(query.c_str());
    return ret == 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "screenrun");

    if(argc > 1) {
        if(strcmp(argv[1], "b") == 0) {
            if(!command_exists("byobu")) {
                ROS_ERROR("Requested byobu for screen, but cannot find byobu executable. Install byobu for this. Falling back to screen for now.");
            } else {
                ROS_INFO("Using byobu for screen.");
                g_screen_cmd = "byobu";
            }
        } else if(strcmp(argv[1], "x") == 0) {
            if(!command_exists("xdotool") || !command_exists("wmctrl")) {
                ROS_FATAL("X Mode requested, but either 'xdotool' or 'wmctrl' are not installed.");
                return 1;
            }
            g_mode = MODE_X;
        }
    }

    ros::NodeHandle nh;

    if(!load())
        return 1;

    std::string wid;
    if(g_mode == MODE_SCREEN) {
        if(screenRunning()) {
            ROS_WARN("Screen \"ros\" already running, reusing this session");
        } else {
            ROS_INFO("Creating screen \"ros\"");
            if(!executeCmd(g_screen_cmd + " -S ros -d -m")) {
                ROS_FATAL("failed");
                return 1;
            }
        }
    } else if(g_mode == MODE_X) {
        // get the active window id
        std::string cmd = "xprop -root | grep \"_NET_ACTIVE_WINDOW(WINDOW)\"| awk '{print $5}'";
        if(!queryCmd(cmd, wid)) {
            ROS_FATAL("wid query failed.");
            return 1;
        }
        printf("%s\n", wid.c_str());
        if(!g_params.load())
            return 1;
    }

    for(vector<ProgramEntry>::iterator it = programs.begin(); it != programs.end(); it++) {
        if(g_mode == MODE_SCREEN)
            it->pushToScreen();
        else if(g_mode == MODE_X)
            it->pushToXWindow(wid);
    }

    return 0;
}

