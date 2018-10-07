//gcc -o bin2csv -l rt bin2csv.c
//./bin2csv  fpga_ram.bin >> fpga_ram.csv

/* bin2hex.c
 * Copyright (C) 2007 Tillmann Werner <tillmann.werner@gmx.de>
 *
 * This file is free software; as a special exception the author gives
 * unlimited permission to copy and/or distribute it, with or without
 * modifications, as long as this notice is preserved.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY, to the extent permitted by law; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * This file is part of the honeytrap tools collection.
 *
 * bin2hex reads a file bytewise and write its hex values to stdout.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>


int main(int argc, char *argv[]) {
        u_char byte0,byte1;
        int r0,r1;
        int retval;
        FILE *file;

        if (argc < 2) {
                fprintf(stderr, "Error - No filename given.\n");
                fprintf(stderr, "gcc -o bin2csv -l rt bin2csv.c\n");
                fprintf(stderr, "./bin2csv  fpga_ram.bin >> fpga_ram.csv\n");
                exit(1);
        }

        /* open file */
        if ((file = fopen(argv[1], "r")) == NULL) {
        fprintf(stderr, "Error - Unable to open file: %s.\n", strerror(errno));
                exit(1);
        }

        errno = 0;
        while((retval = fscanf(file, "%c%c", &byte0 ,&byte1)) > 0){

            if(byte0>127) r0=-(256-byte0);
            else r0=byte0;
            if(byte1>127) r1=-(256-byte1);
            else r1=byte1;

            fprintf(stdout, "30-12-2015 17:13:24;Tcpu;36.86;uo;%03d;io;%03d;io;%03d;io;%03d;io;%03d;io;%03d;io;%03d;io;%03d\n",
                    r0,r1,r1,r1,r1,r1,r1,r1);
        }
        if ((retval = EOF) && errno) fprintf(stderr, "Error - Unable to read from file: %s.\n", strerror(errno));

        fclose(file);
        return(0);
}
