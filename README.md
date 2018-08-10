# Platooning

10.08.18

The repository contains code and datasets of vehicle platooning scenarios to be investigated via machine learning [1]. 

A presentation introduces the problem with some examples. 

Two simulation environments are used: 1. differential equations of [2] and 2. plexe. 
1. is for string stability of the platoon. 
2. deals with collision prediction. 

Code of the simulation 1. is reported in the repository. 2. is based on PLexe (plexe.car2x.org/) and has been automated to generate several runs under different system parameters; the inherent code is reported (Plexe automation.rar).

Plexe automation.rar deals with running Plexe under different settings of the parameters. The main script is Plexe-Automatic.sh. The inherent files called by the script are reported for the sake of clarity.

Thanks to Alessandro Fermi (a.fermi@rulex.ai) for Plexe automation.

Do not hesitate to contact me: maurizio.mongelli@ieiit.cnr.it.

[1] A. Fermi, M. Mongelli, M. Muselli and E. Ferrari, "Identification of safety regions in vehicle platooning via machine learning," 2018 14th IEEE International Workshop on Factory Communication Systems (WFCS), Imperia, 2018, pp. 1-4, doi: 10.1109/WFCS.2018.8402372.

[2] L. Xu, L. Y.Wang, G. Yin, H. Zhang. “Communication information structures and contents for enhanced safety of highway vehicle platoons”, IEEE Transactions on Vehicular Technology, 63(9), pp. 4206–4220, (Nov 2014).
