/*

	*** MAIN PROGRAM: (cu x inseamna ca am inteles) ***
	ORDER OF OPERATIONS: 

	1) x - citeste switchuri (218, declaration: 391)
	2) x - reset camera (233 - 236)
	3) x  - grab camera frame (239, declaration: 413)
	4) derivate line scan (248, declaration: 522)
	5) adjust lights 
	6) find edges
	7) review edges
	8) act on track
	9) feedback lights
	10) speed control
	11) drive
	12) final: dump data

	_______________________________
	INSTRUCTIONS:

	- dumpData: printezi toate frameurile care au fost inregistrare de masina.
	- captureData: incarca fiecare frame cu informatii:
		* index
		* line position
		* steer setting
		* left/right setting
		* max nr. of frames = macro.
	- readSwitches:
		* switch 2 ON/OFF: log data/don't log.
		* switch 3 ON/OFF: risky driving/ don't.
		* switch 4 ON/OFF: calibrate start/stop gate.
	- daca switch 2 e ON si go = false (?), atunci tii apasat B si vedem datele.
		-> vezi cand go devine false in MCP
	- terminal output: 	* 0 = no terminal output
						* 1 = frame rate only
						* 2 = time of operations
						* 3 = output with delay (?)
	- pentru a porni/opri: buton A
*/