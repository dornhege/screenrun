#include <ros/ros.h>
#include <stdio.h>
#include <dirent.h>
#include <stdlib.h>
#include <vector>
#include <string>
using namespace std;

string g_screen_cmd = "screen";

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "screenrun");

    if(argc > 1) {
        if(strcmp(argv[1], "b") == 0) {
            ROS_INFO("Using byobu for screen.");
            g_screen_cmd = "byobu";
        }
    }

    ros::NodeHandle nh;
    
    if(!load())
        return 1;

    if(screenRunning()) {
        ROS_WARN("Screen \"ros\" already running, reusing this session");
    } else {
        ROS_INFO("Creating screen \"ros\"");
        if(!executeCmd(g_screen_cmd + " -S ros -d -m")) {
            ROS_FATAL("failed");
            return 1;
        }
    }

    for(vector<ProgramEntry>::iterator it = programs.begin(); it != programs.end(); it++) {
        it->pushToScreen();
    }

    return 0;
}
