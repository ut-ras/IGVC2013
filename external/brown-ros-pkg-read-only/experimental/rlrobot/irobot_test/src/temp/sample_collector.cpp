
/*
 *
 *
 */
#include <sample_collector.h>

SampleCollector::SampleCollector()
{
  maxSteps=50;
  maxEpisodes=100;

}

SampleCollector::SampleCollector(int numstep, int maxep)
{
  maxSteps=numstep;
  maxEpisodes=maxep;
}

SampleCollector::SampleCollector(int numstep, int maxep, Policy pol)
{
  maxSteps=numstep;
  maxEpisodes=maxep;
  policy=pol;
}
