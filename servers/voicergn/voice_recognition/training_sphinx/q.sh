#!/bin/sh

./bw \
 -hmmdir hub4wsj_sc_8k \
 -moddeffn hub4wsj_sc_8k/mdef.txt \
 -ts2cbfn .semi. \
 -feat 1s_c_d_dd \
 -svspec 0-12/13-25/26-38 \
 -cmn current \
 -agc none \
 -dictfn a.dic \
 -ctlfn a.fileids \
 -lsnfn a.transcription \
 -accumdir .