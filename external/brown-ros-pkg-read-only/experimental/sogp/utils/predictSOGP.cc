/*Predict from an SOGP file

*/

#include "sogp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int main(int argc,const char **argv){
	if(argc<2){
		printf("Usage:  predictRoger -s SOGPfile -i inputFile indim [-m {trans | mm | ms | f stateFile}] -o predictedOutputFile \n");
		printf("Or help for more detailed Roger parameters\n");
		exit(-1);
	}
	
	int pdim=-1;
	FILE *rfile=NULL,*ifile=NULL,*sofile=NULL,*sfile=NULL;
	SOGP *m_sogp=NULL;

	for(int argi=1;argi<argc;argi++){
		if(!strcmp(argv[argi],"-s")){
			if(!(rfile = fopen(argv[++argi],"r")))
				perror("Error opening SOGPfile:\n");
			printf("Reading SOGP from %s\n",argv[argi]);
			m_sogp = new SOGP();
			bool read = m_sogp->readFrom(rfile);
			if(!read)
				perror("Error loading SOGPfile:\n");
		}
		if(!strcmp(argv[argi],"-i")){
			if(!(ifile = fopen(argv[++argi],"r")))
				perror("Error opening inputfile:\n");
			printf("Reading input from %s, with dimensionality ",argv[argi]);
			pdim = atoi(argv[++argi]);
			printf("%d\n",pdim);
		}
		if(!strcmp(argv[argi],"-w")){
			if(!(sofile = fopen(argv[++argi],"w")))
				perror("Error opening state outputfile:\n");
			printf("Saving state to %s\n",argv[argi]);
		}
		if(!strcmp(argv[argi],"-o")){
			if(!(sfile = fopen(argv[++argi],"w")))
				perror("Error opening output savefile:\n");
		}
	}
	


	double p = 0;
	double sig = 0;
	int cnt = 0;
	ColumnVector ins(pdim),outs;
	while(fscanf(ifile,"%lf ",&p)!=EOF){
		ins(1) = p;
		for(int i=1;i<pdim;i++){
			fscanf(ifile,"%lf ",&(p));
			ins(i+1) = p;
		}
		outs = m_sogp->predict(ins, sig);
		for(int i = 1; i <= outs.Nrows(); i++){
			fprintf(sfile,"%lf ",outs(i));
		}
		fprintf(sfile,"\n");
		cnt++;
	}
	fclose(sfile);
	fclose(ifile);
}
