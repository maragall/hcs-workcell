"""
The one-time setup of the arm, as a terminal program: `python cli.py setup`. It ends with READY.

    runner      Step, Section, Runner: print each line, wait for Enter on the user's, stop on the first refusal
    sections    the sections in order: connect (the arm found, connected and enabled; every Squid answering), places (the hotel
                and the module shown by hand), preset (the arm's program written and uploaded)
    cell        the arm and the positions file; discover: finds the arm on the network and asks Squid; doctor: the read-only
                check of the PC; sim: the fakes for --sim
"""
