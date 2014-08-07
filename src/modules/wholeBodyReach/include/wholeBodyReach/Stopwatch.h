/*
 Copyright (c) 2010-2013 Tommaso Urli       tommaso.urli@uniud.it     University of Udine
               2014      Andrea Del Prete   andre.delprete@gmail.com  LAAS-CNRS

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/


#ifndef WBR_STOPWATCH_H
#define WBR_STOPWATCH_H

#include "wholeBodyReach/Stdafx.h"

#ifndef WIN32
	/* The classes below are exported */
	#pragma GCC visibility push(default)
#endif

// if DO_PROFILING is defined then the profiling is active
#define DO_PROFILING

// Some handy macros for using the StopWatch class to profile your software.
// In this way the profiling can be easily disabled.
#ifdef DO_PROFILING
    #define PROFILE(name,code)      getProfiler.start(name); code getProfiler().stop(name)
    #define START_PROFILING(name)   getProfiler().start(name)
    #define STOP_PROFILING(name)    getProfiler().stop(name)
    #define PRINT_PROFILING_INFO    getProfiler().report_all()
#else
    #define PROFILE(name,code)
    #define START_PROFILING(name)
    #define STOP_PROFILING(name)
    #define PRINT_PROFILING_INFO
#endif


enum StopwatchMode
{
	NONE		= 0,	// Clock is not initialized
	CPU_TIME	= 1,	// Clock calculates time ranges using ctime and CLOCKS_PER_SEC
	REAL_TIME	= 2,	// Clock calculates time by asking the operating system how much real time passed
};

/** Struct to hold the performance data */
class PerformanceData
{
public:
    PerformanceData() :
    clock_start(0),
    total_time(0),
    min_time(0),
    max_time(0),
    last_time(0),
    paused(false),
    stops(0)
    {}
    
    long double	clock_start;        /** Start time */
    long double	total_time;         /** Cumulative total time */
    long double	min_time;           /** Minimum time */
    long double	max_time;           /** Maximum time */
    long double last_time;          /** Last time */
    bool paused;                    /** Tells if this performance has been paused, only for internal use */
    int	stops;                      /** How many cycles have been this stopwatch executed? */
};

/**
	@brief A class representing a stopwatch.

		@code
		Stopwatch swatch();
		@endcode

	The Stopwatch class can be used to measure execution time of code, algorithms, etc., the Stopwatch can
	be initialized in two time-taking modes, CPU time and real time:
	
		@code
		swatch.set_mode(REAL_TIME);
		@endcode

	CPU time is the time spent by the processor on a certain piece of code, while real time is the real
	amount of time taken by a certain piece of code to execute (i.e. in general if you are doing hard work
	such as image or video editing on a different process the measured time will probably increase).
	
	How does it work? Basically, one wraps the code to be measured with the following method calls:
	
		@code
		swatch.start("My astounding algorithm");
		// Hic est code
		swatch.stop("My astounding algorithm");
		@endcode
		
	A string representing the code ID is provided so that nested portions of code can be profiled separately:
	
		@code
		swatch.start("My astounding algorithm");
		
		swatch.start("My astounding algorithm - Super smart init");
		// Initialization
		swatch.stop("My astounding algorithm - Super smart init");
		
		swatch.start("My astounding algorithm - Main loop");
		// Loop
		swatch.stop("My astounding algorithm - Main loop");
		
		swatch.stop("My astounding algorithm");
		@endcode

	Note: ID strings can be whatever you like, in the previous example I have used "My astounding algorithm - *"
	only to enforce the fact that the measured code portions are part of My astounding algorithm, but there's no
	connection between the three measurements.
		
	If the code for a certain task is scattered through different files or portions of the same file one can use 
	the start-pause-stop method:
	
		@code
		swatch.start("Setup");
		// First part of setup
		swatch.pause("Setup");
		
		swatch.start("Main logic");
		// Main logic
		swatch.stop("Main logic");
		
		swatch.start("Setup");
		// Cleanup (part of the setup)
		swatch.stop("Setup");
		@endcode
		
	Finally, to report the results of the measurements just run:
	
		@code
		swatch.report("Code ID");
		@endcode

	Thou can also provide an additional std::ostream& parameter to report() to redirect the logging on a different
	output. Also, you can use the get_performance_info() method to get all the numeric data, without
	all the details of the logging. You can also extend Stopwatch to implement your own logging syntax.
		
	To report all the measurements:
	
		@code
		swatch.report_all();
		@endcode

	Same as above, you can redirect the output by providing a std::ostream& parameter.		

*/
class Stopwatch
{
protected:
    
	StopwatchMode mode;     /** Time taking mode */
    bool active;            /** Flag to hold the clock's status */

	/** Pointer to the dynamic structure which holds the collection of performance data */
	std::map<std::string, PerformanceData >* records_of;
    
public:
	
	/** Constructor.
     * @param _mode It specifies how time is measured.
     */
	Stopwatch(StopwatchMode _mode=NONE);

	/** Destructor */
	~Stopwatch()
    { delete records_of; }

	/** Tells if a performance with a certain name exists.
     * @param perf_name String associated to the performance.
     * @return True if a performance with the given name exists, false otherwise.
     */
	bool performance_exists(std::string perf_name)
    { return (records_of->find(perf_name) != records_of->end()); }


	/** Initialize stopwatch to use a certain time taking mode 
     * @param mode The modality used to measure time.
     */
	void set_mode(StopwatchMode new_mode)
    { mode = new_mode; }


	/** Start the stopwatch related to a certain piece of code
     * @param perf_name The string ID associated to the piece of code to profile.
     */
	void start(std::string perf_name);
		
	/** Stops the stopwatch related to a certain piece of code.
     * @param perf_name The string ID associated to the piece of code to profile.
     * @return False if perf_name is not recognized, true otherwise.
     */
	bool stop(std::string perf_name);
	
	/** Pauses the stopwatch related to a certain piece of code.
     * @param perf_name The string ID associated to the piece of code to profile.
     * @return False if perf_name is not recognized, true otherwise.
     */
	bool pause(std::string perf_name);

	/** Reset a certain performance record.
     * @param perf_name The string ID associated to the piece of code to profile.
     * @return False if perf_name is not recognized, true otherwise.
     */
	bool reset(std::string perf_name);
		
	/** Resets all the performance records. */
	void reset_all();
	
	/** Dump the data of a certain performance record.
     * @param perf_name The string ID associated to the piece of code to profile.
     * @param precision The number of digits printed after the comma.
     * @param output The output stream on which to print the timing information.
     * @return False if perf_name is not recognized, true otherwise.
     */
	bool report(std::string perf_name, int precision=2, std::ostream& output = std::cout);

	/** Dump the data of all the performance records.
     * @param precision The number of digits printed after the comma.
     * @param output The output stream on which to print the timing information.
     */
	void report_all(int precision=2, std::ostream& output = std::cout);

    /** Gets all the data of the specified performance.
     * @param perf_name The string ID associated to the piece of code to profile.
     * @param data The output data.
     * @return False if perf_name is not recognized, true otherwise.
     */
    bool get_performance_data(std::string perf_name, PerformanceData& data);

	/**	Turn off clock, all the Stopwatch::* methods return without doing anything after this method is called. */
	void turn_off()
    { active = false; }
	
	/** Turn on clock, restore clock operativity after a turn_off(). */
	void turn_on()
    { active = true; }
	
	/** Take time, depends on mode */
	long double take_time();
};

Stopwatch& getProfiler();

#ifndef WIN32
	#pragma GCC visibility pop
#endif

#endif
