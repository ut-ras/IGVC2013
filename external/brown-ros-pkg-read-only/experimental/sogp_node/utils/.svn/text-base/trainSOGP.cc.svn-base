/*Train an SOGP from a file

*/

#include "sogp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int main(int argc,const char **argv){
	if(argc<2){
		printf("Usage:  trainSOGP -i inputfile indim -o outputfile outdim -s savefile\n");
		printf("Or help for more detailed SOGP parameters\n");
		exit(-1);
	}

	int pdim=-1,adim=-1;
	FILE *ifile=NULL,*ofile=NULL,*sfile=NULL;
	SOGP *m_sogp=NULL;
	unsigned int seed = time(NULL);
	double w=.1;
	double n=.1;
	int cap = 300;

	for(int argi=1;argi<argc;argi++){
		if(!strcmp(argv[argi],"-i")){
			if(!(ifile = fopen(argv[++argi],"r")))
				perror("Error opening inputfile:\n");
			printf("Reading input from %s, with dimensionality ",argv[argi]);
			pdim = atoi(argv[++argi]);
			printf("%d\n",pdim);
		}
		if(!strcmp(argv[argi],"-o")){
			if(!(ofile = fopen(argv[++argi],"r")))
				perror("Error opening outputfile:\n");
			printf("Reading output from %s, with dimensionality ",argv[argi]);
			adim = atoi(argv[++argi]);
			printf("%d\n",adim);
		}
		if(!strcmp(argv[argi],"-s")){
			if(!(sfile = fopen(argv[++argi],"w")))
				perror("Error opening savefile:\n");
		}

		
		if(!strcmp(argv[argi],"help")){
			printf("SOGP options are: [-c capacity -w width -n noise]\n");
			return false;
		}

		if(!strcmp(argv[argi],"-c"))
			cap=atoi(argv[++argi]);
		if(!strcmp(argv[argi],"-w"))
			w=atof(argv[++argi]);
		if(!strcmp(argv[argi],"-n"))
			n=atof(argv[++argi]);

	}

	printf("Starting an SOGP with w = %lf, n = %lf and capacity %d\n",w,n,cap);
	RBFKernel kern(w);
	SOGPParams params(&kern);
	params.s20=n;
	params.capacity=cap;
 
	
	m_sogp = new SOGP(w,n,cap);

	ColumnVector ins(pdim),outs(adim);
	double p;
	double a;

	srand(seed);
	int cnt=0;
	time_t start=time(NULL);
	while(fscanf(ifile,"%lf ",&p)!=EOF){
		ins(1) = p;
		for(int i=1;i<pdim;i++){
			fscanf(ifile,"%lf ",&(p));
			ins(i+1) = p;
		}
		for(int i=0;i<adim;i++){
			fscanf(ofile,"%lf ",&(a));
			outs(i+1) = a;
		}
		m_sogp->add(ins,outs);
		fflush(stdout);//For when using Tee
		cnt++;
	}
	time_t end = time(NULL);
	long int hour=0,min=0,sec=end-start;
	min = sec/60;
	sec = sec%60;
	hour = min/60;
	min = min%60;
	printf("Trained on %d points with seed %d in %2ld:%2ld:%2ld time\n",cnt,seed,hour,min,sec);

	m_sogp->printTo(sfile);
	fclose(sfile);
	fclose(ifile);
	fclose(ofile);
}
