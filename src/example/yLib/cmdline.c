#include <string.h>
#include "cmdline.h"
#include <stdint.h>
#include "ethopts.h"
#include "yInc.h"



//extern const char *StringFromFresult(FRESULT fresult); //sdcard.c
extern void printf(const char *pcString, ...);

//Mdio
#if 0
//ssd
extern int Cmd_FPGAread(int argc, char *argv[]);
extern int Cmd_FPGAwrite(int argc, char *argv[]);
//int 	   dtCmd_help(int argc, char *argv[]);
extern int dtCmd_SSDls(int argc, char *argv[]);
extern int dtCmd_SSDcd(int argc, char *argv[]);
extern int dtCmd_SSDpwd(int argc, char *argv[]);
extern int dtCmd_SSDcat(int argc, char *argv[]);
#endif

#if (CMDLINE_USE_FOR == CMDLINE_MDIO)
//Mdio
extern int stmCmd_help(int argc, char *argv[]);
extern int stmMdioBitbangCmd_read(int argc, char *argv[]);
extern int stmMdioBitbangCmd_readAll(int argc, char *argv[]);
extern int stmMdioBitbangCmd_SetPhyAddrAndInitConfig(int argc, char *argv[]);
//extern int dtMdioCmd_readSQI(int argc, char *argv[]);
extern int stmMdioBitbangCmd_write(int argc, char *argv[]);
#elif (CMDLINE_USE_FOR == CMDLINE_MDIO407_107)
//Mdio
extern int stmMdio407_107Cmd_help(int argc, char *argv[]);
extern int stmMdio407_107Cmd_read(int argc, char *argv[]);
extern int stmMdio407_107Cmd_readAll(int argc, char *argv[]);
//extern int dtMdioCmd_readSQI(int argc, char *argv[]);
extern int stmMdio407_107Cmd_write(int argc, char *argv[]);
extern int stmMdio407_107Cmd_SetPhyAddrAndInitConfig(int argc, char *argv[]);
extern int stmMdio407_107Cmd_SetPhyClkOutput(int argc, char *argv[]);
#elif (CMDLINE_USE_FOR == CMDLINE_SJA1105)
extern int stmSja1105Cmd_help(int argc, char *argv[]);
extern int stmSja1105Cmd_config(int argc, char *argv[]);
extern int stmSja1105Cmd_status(int argc, char *argv[]);
extern int stmSja1105Cmd_read_reg(int argc, char *argv[]);
extern int stmSja1105Cmd_write_reg(int argc, char *argv[]);

#elif (CMDLINE_USE_FOR == CMDLINE_RSTP)

#endif

#if 0
//ssd
extern int Cmd_FPGAread(int argc, char *argv[]);
extern int Cmd_FPGAwrite(int argc, char *argv[]);
//int 	   dtCmd_help(int argc, char *argv[]);
extern int dtCmd_SSDls(int argc, char *argv[]);
extern int dtCmd_SSDcd(int argc, char *argv[]);
extern int dtCmd_SSDpwd(int argc, char *argv[]);
extern int dtCmd_SSDcat(int argc, char *argv[]);
#endif

// The buffer that holds the command line.
char g_cCmdBuf[CMD_BUF_SIZE];
// Defines the maximum number of arguments that can be parsed.
#ifndef CMDLINE_MAX_ARGS
#define CMDLINE_MAX_ARGS        5
#endif
//*****************************************************************************
//
// This is the table that holds the command names, implementing functions,
// and brief description.
//
//*****************************************************************************
#if 0
tCmdLineEntry g_sCmdTable[] =
{
    { "help",   dtCmd_help,      " : Display list of commands" },
    { "h",      dtCmd_help,   "    : alias for help" },
    { "?",      dtCmd_help,   "    : alias for help" },
    { "r",      Cmd_FPGAread,   "  : Read Register in FPGA" },
    { "w",      Cmd_FPGAwrite,   " : Write Register on FPGA" },
    { "ls",     dtCmd_SSDls,      "  : Display list of files" },
    { "chdir",  dtCmd_SSDcd,         ": Change directory" },
    { "cd",     dtCmd_SSDcd,      "   : alias for chdir" },
    { "pwd",    dtCmd_SSDpwd,      "  : Show current working directory" },
    { "cat",    dtCmd_SSDcat,      "  : Show contents of a text file" },
    { 0, 0, 0 }
};
#endif
#if (CMDLINE_USE_FOR == CMDLINE_MDIO)
#if (MDIO_FOR == EDU)
// For MDIO
tCmdLineEntry g_sCmdTable[] =
{
    { "help",   stmCmd_help,      " : Display list of commands" },
    { "h",      stmCmd_help,   "    : alias for help" },
    { "?",      stmCmd_help,   "    : alias for help" },
    { "r",      stmMdioBitbangCmd_read,   "  : read <reg addr>" },
    { "ra",     stmMdioBitbangCmd_readAll,   " : readAllbasic" },
    { "sa",     stmMdioBitbangCmd_SetPhyAddrAndInitConfig,   " : setPHYAD <phy addr>" }, //45=clause45, 4522=Annex22D
 //   { "rs",     dtMdioCmd_readSQI,   " : read <phy addr>" },
    { "w",      stmMdioBitbangCmd_write,   " : write <reg addr> <value>" },
    { 0, 0, 0 }
};
#elif (MDIO_FOR == MDIO_FOR_REALTEK9K)
extern int RTL9Kmc_Cmd_help(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_read(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_readAll(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_write(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_setms(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_showstatus(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_softreset(int argc, char *argv[]);
// For MDIO
tCmdLineEntry g_sCmdTable[] =
{
    { "help",   RTL9Kmc_Cmd_help,      " : Display list of commands" },
    { "h",      RTL9Kmc_Cmd_help,   "    : alias for help" },
    { "?",      RTL9Kmc_Cmd_help,   "    : alias for help" },
    { "r",      RTL9Kmc_Cmd_read,   "  : read <phy addr> <reg addr> <page>" },
    { "ra",     RTL9Kmc_Cmd_readAll,   " : readAll <phy addr>" },
    { "w",      RTL9Kmc_Cmd_write,   " : write <phy addr> <reg addr> <page> <value>" },
    { "ms",     RTL9Kmc_Cmd_setms,   " : set as master/slave(RTL9K) <1=master/0=slave>" },
    { "ss",     RTL9Kmc_Cmd_showstatus,   " : showstatus <phy addr>" },
    { "sr",     RTL9Kmc_Cmd_softreset,   " : softreset <phy addr>" },
    { 0, 0, 0 }
};
#else
//For ov7670 camera
extern int cf7670Cmd_help(int argc, char *argv[]);
extern void cf7670_doCmd(int argc, char *argv[]);
tCmdLineEntry g_sCmdTable[] =
{
    { "help",   cf7670Cmd_help,      " : Display list of commands" },
    { "h",      cf7670Cmd_help,   "    : alias for help" },
    { "?",      cf7670Cmd_help,   "    : alias for help" },
    { "c",      cf7670_doCmd,   "  : command <i,0,l,...> <arg2>" },
//  { "t",      dtMdioCmd_readSpan,   " : take picture(toggle)" },
//  { "x",      dtMdioCmd_write,   " : write <phy addr> <reg addr> <value>" },
    { 0, 0, 0 }
};
#endif
#elif(CMDLINE_USE_FOR == CMDLINE_MDIO407_107)
#if (MDIO_FOR == MDIO_FOR_EDU)
// For MDIO
tCmdLineEntry g_sCmdTable[] =
{
    { "help",   stmMdio407_107Cmd_help,      " : Display list of commands" },
    { "h",      stmMdio407_107Cmd_help,   "    : alias for help" },
    { "?",      stmMdio407_107Cmd_help,   "    : alias for help" },
    { "r",      stmMdio407_107Cmd_read,   "  : read <reg addr>" },
    { "ra",     stmMdio407_107Cmd_readAll,   " : readAllbasic" },
    { "sa",     stmMdio407_107Cmd_SetPhyAddrAndInitConfig,   " : setPHYAD <phy addr>" }, //45=clause45, 4522=Annex22D
 //   { "rs",     dtMdioCmd_readSQI,   " : read <phy addr>" },
    { "w",      stmMdio407_107Cmd_write,   " : write <reg addr> <value>" },
    { "c",      stmMdio407_107Cmd_SetPhyClkOutput, " : en/dis Clk25OutputToPhy(1/0)"},
    { 0, 0, 0 }
};
#elif (MDIO_FOR == MDIO_FOR_REALTEK9K)
extern int RTL9Kmc_Cmd_help(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_read(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_readAll(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_write(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_setms(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_showstatus(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_softreset(int argc, char *argv[]);
// For MDIO
tCmdLineEntry g_sCmdTable[] =
{
    { "help",   RTL9Kmc_Cmd_help,      " : Display list of commands" },
    { "h",      RTL9Kmc_Cmd_help,   "    : alias for help" },
    { "?",      RTL9Kmc_Cmd_help,   "    : alias for help" },
    { "r",      RTL9Kmc_Cmd_read,   "  : read <phy addr> <reg addr> <page>" },
    { "ra",     RTL9Kmc_Cmd_readAll,   " : readAll <phy addr>" },
    { "w",      RTL9Kmc_Cmd_write,   " : write <phy addr> <reg addr> <page> <value>" },
    { "ms",     RTL9Kmc_Cmd_setms,   " : set as master/slave(RTL9K) <1=master/0=slave>" },
    { "ss",     RTL9Kmc_Cmd_showstatus,   " : showstatus <phy addr>" },
    { "sr",     RTL9Kmc_Cmd_softreset,   " : softreset <phy addr>" },
    { 0, 0, 0 }
};
#else
//For ov7670 camera
extern int cf7670Cmd_help(int argc, char *argv[]);
extern void cf7670_doCmd(int argc, char *argv[]);
tCmdLineEntry g_sCmdTable[] =
{
    { "help",   cf7670Cmd_help,      " : Display list of commands" },
    { "h",      cf7670Cmd_help,   "    : alias for help" },
    { "?",      cf7670Cmd_help,   "    : alias for help" },
    { "c",      cf7670_doCmd,   "  : command <i,0,l,...> <arg2>" },
//  { "t",      dtMdioCmd_readSpan,   " : take picture(toggle)" },
//  { "x",      dtMdioCmd_write,   " : write <phy addr> <reg addr> <value>" },
    { 0, 0, 0 }
};
#endif

#elif(CMDLINE_USE_FOR == CMDLINE_SJA1105)

extern int cf7670Cmd_help(int argc, char *argv[]);
extern stmSja1105Cmd_config_new (int argc, char *argv[]);
extern stmSja1105Cmd_config_load(int argc, char *argv[]);
extern stmSja1105Cmd_config_save(int argc, char *argv[]);
extern stmSja1105Cmd_config_default(int argc, char *argv[]);
extern stmSja1105Cmd_config_modify(int argc, char *argv[]);
extern stmSja1105Cmd_config_upload(int argc, char *argv[]);
extern stmSja1105Cmd_config_show(int argc, char *argv[]);
extern stmSja1105Cmd_config_hexdump(int argc, char *argv[]);
extern stmSja1105Cmd_status_general(int argc, char *argv[]);
extern stmSja1105Cmd_status_port(int argc, char *argv[]);
extern stmSja1105Cmd_status_vl(int argc, char *argv[]);
extern stmSja1105Cmd_read_reg(int argc, char *argv[]);
extern stmSja1105Cmd_write_reg(int argc, char *argv[]);
extern stmSja1105Cmd_reset_warm(int argc, char *argv[]);
extern stmSja1105Cmd_reset_cold(int argc, char *argv[]);

tCmdLineEntry g_sCmdTable[] =
{
    { "help",   stmSja1105Cmd_help,      " : Display list of commands" },
    { "h",      stmSja1105Cmd_help,   "    : alias for help" },
    { "?",      stmSja1105Cmd_help,   "    : alias for help" },
    { "cn",     stmSja1105Cmd_config_new, "    : config new"}, //printf("* new [-d|--device-id <value>], default 0x9e00030e (SJA1105TEL)\r\n");
    { "cl",     stmSja1105Cmd_config_load, "    : config load(read xml config file, unpack it, also flash)"},//    	printf("* load [-f|--flash] <filename.xml>\r\n");
    { "cs",     stmSja1105Cmd_config_save, "    : config save"},//printf("* save <filename.xml>\r\n");
    { "cd",     stmSja1105Cmd_config_default, "    : config default(read default binary config file, unpack it)"},//printf("* default [-f|--flash] <config>, which can be ls1021atsn - load a built-in config compatible with the NXP LS1021ATSN board\r\n");
    { "cm",     stmSja1105Cmd_config_modify, "    : config modify"},//printf("* modify [-f|--flash] <table>[<entry_index>] <field> <value>\r\n");
    { "cu",     stmSja1105Cmd_config_upload, "    : config upload(flash the switch with the binary config file)"},//printf("* upload\r\n");
    { "ch",     stmSja1105Cmd_config_show, "    : config show"}, //printf("* show [<table>]. If no table is specified, shows entire config.\r\n");
    { "cx",     stmSja1105Cmd_config_hexdump, "    : config hexdump"}, //printf("* hexdump [<table>]. If no table is specified, dumps entire config.\r\n");
    { "sg",     stmSja1105Cmd_status_general, "    : status general"},//General Status Information Register\n");
    { "sp",     stmSja1105Cmd_status_port, "    : status port [c] [-1]"},//printf(" * port    -> Port status Information Register\n" \    "              Provide Port No. as argument [0-4]\n");
    { "sv",     stmSja1105Cmd_status_vl, "    : status virtual link [c]"},
    { "rr",     stmSja1105Cmd_read_reg,   "  : reg read <reg addr>" },//printf(" * sja1105-tool reg <address> [<write_value>] :"     "Read or write register\n");
    { "rw",     stmSja1105Cmd_write_reg,  " : reg write <reg addr>" },
    { "rd",     stmSja1105Cmd_write_reg,  " : reg dump <reg addr> <count>" },//printf(" * sja1105-tool reg dump <address> <count>    :"     "Incrementally dump registers starting at the given address\n");
    { "rsw",     stmSja1105Cmd_reset_warm,  " : reset warm" },//printf(" * sja1105-tool reset warm\n");
    { "rsc",     stmSja1105Cmd_reset_cold,  " : reset cold" },//printf(" * sja1105-tool reset cold\n");

//    { "ra",     stmSja1105Cmd_readAll,   " : readAllbasic" },
    //{ "wr",     stmSja1105Cmd_SetPhyAddrAndInitConfig,   " : setPHYAD <phy addr>" }, //45=clause45, 4522=Annex22D
//    { "w",      stmSja1105Cmd_write,   " : write <reg addr> <value>" },
    { 0, 0, 0 }
};

#elif (CMDLINE_USE_FOR == CMDLINE_RSTP)
extern int stmCmd_help(int argc, char *argv[]);
extern int stmRstpCmd_read(int argc, char *argv[]);
extern int stmRstpCmd_readAll(int argc, char *argv[]);
extern int stmRstpCmd_write(int argc, char *argv[]);
extern int stmRstpCmd_showStatus(int argc, char *argv[]);
extern int stmRstpCmd_softReset(int argc, char *argv[]);
extern tCmdLineEntry g_sCmdTable[];
#endif

//*****************************************************************************
//
//! Process a command line string into arguments and execute the command.
//!
//! \param pcCmdLine points to a string that contains a command line that was
//! obtained by an application by some means.
//!
//! This function will take the supplied command line string and break it up
//! into individual arguments.  The first argument is treated as a command and
//! is searched for in the command table.  If the command is found, then the
//! command function is called and all of the command line arguments are passed
//! in the normal argc, argv form.
//!
//! The command table is contained in an array named <tt>g_sCmdTable</tt> which
//! must be provided by the application.
//!
//! \return Returns \b CMDLINE_BAD_CMD if the command is not found,
//! \b CMDLINE_TOO_MANY_ARGS if there are more arguments than can be parsed.
//! Otherwise it returns the code that was returned by the command function.
//
//*****************************************************************************
int
CmdLineProcess(char *pcCmdLine)
{
    static char *argv[CMDLINE_MAX_ARGS + 1];
    char *pcChar;
    int argc;
    int bFindArg = 1;
    tCmdLineEntry *pCmdEntry;

    //
    // Initialize the argument counter, and point to the beginning of the
    // command line string.
    //
    argc = 0;
    pcChar = pcCmdLine;

    //
    // Advance through the command line until a zero character is found.
    //
    while(*pcChar)
    {
        //
        // If there is a space, then replace it with a zero, and set the flag
        // to search for the next argument.
        //
        if(*pcChar == ' ')
        {
            *pcChar = 0;
            bFindArg = 1;
        }

        //
        // Otherwise it is not a space, so it must be a character that is part
        // of an argument.
        //
        else
        {
            //
            // If bFindArg is set, then that means we are looking for the start
            // of the next argument.
            //
            if(bFindArg)
            {
                //
                // As long as the maximum number of arguments has not been
                // reached, then save the pointer to the start of this new arg
                // in the argv array, and increment the count of args, argc.
                //
                if(argc < CMDLINE_MAX_ARGS)
                {
                    argv[argc] = pcChar;
                    argc++;
                    bFindArg = 0;
                }

                //
                // The maximum number of arguments has been reached so return
                // the error.
                //
                else
                {
                    return(CMDLINE_TOO_MANY_ARGS);
                }
            }
        }

        //
        // Advance to the next character in the command line.
        //
        pcChar++;
    }

    //
    // If one or more arguments was found, then process the command.
    //
    if(argc)
    {
        //
        // Start at the beginning of the command table, to look for a matching
        // command.
        //
        pCmdEntry = &g_sCmdTable[0];

        //
        // Search through the command table until a null command string is
        // found, which marks the end of the table.
        //
        while(pCmdEntry->pcCmd)
        {
            //
            // If this command entry command string matches argv[0], then call
            // the function for this command, passing the command line
            // arguments.
            //
            if(!strcmp(argv[0], pCmdEntry->pcCmd))
            {
                return(pCmdEntry->pfnCmd(argc, argv));
            }

            //
            // Not found, so advance to the next entry.
            //
            pCmdEntry++;
        }
    }else {
		return(CMDLINE_NO_ARGS);
	}



    //
    // Fall through to here means that no matching command was found, so return
    // an error.
    //
    return(CMDLINE_BAD_CMD);
}


//*****************************************************************************
// This function implements the "help" command.  It prints a simple list
// of the available commands with a brief description.
//*****************************************************************************
int stmCmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    printf("\r\nAvailable commands\r\n");
    printf("------------------\r\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_sCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The
    // end of the table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        printf("%s%s\r\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}

void stmCmd_Init()
{
     // Print hello message to user.
#if (MDIO_FOR == MDIO_FOR_EDU)
    printf("Datong CLI Setting..\r\n");
#elif (MDIO_FOR == MDIO_FOR_REALTEK9K)
    printf("RTL9K CLI Setting..\r\n");
#else
    printf("Datong CLI Setting..\r\n");
#endif
    printf("Type \'help\' for help.\r\n");
    printf("CLI> ");
}
/*
void dtCmd_handler()//
{
    	int nStatus;
    	unsigned char showCmdPrompt = 0;

        // Get a line of text from the user.
        //

    	if(UARTPeek('\r') != -1){ //BUFFERED MODE ONLY
    		UARTgets(g_cCmdBuf, sizeof(g_cCmdBuf));
     	}
    	else
    		return ;

        //
        // Pass the line from the user to the command processor.
        // It will be parsed and valid commands executed.
        //
        nStatus = CmdLineProcess(g_cCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(nStatus == CMDLINE_BAD_CMD)
        {
            printf("Bad command!\r\n");
        }

        //
        // Handle the case of too many arguments.
        //
        else if(nStatus == CMDLINE_TOO_MANY_ARGS)
        {
            printf("Too many arguments for command processor!\r\n");
        }else if(nStatus == CMDLINE_NO_ARGS){
            // Print a prompt to the console.  Show the CWD.
        	printf("\r\nCLI> ");
        	return;
        }
        //
        // Otherwise the command was executed.  Print the error
        // code if one was returned.
        //
        else if(nStatus != 0)
        {
            //printf("Command returned error code %s\r\n",StringFromFresult((FRESULT)nStatus));
        }
        // Print a prompt to the console.  Show the CWD.
        //
    	printf("\r\nCLI> ");
}
*/
