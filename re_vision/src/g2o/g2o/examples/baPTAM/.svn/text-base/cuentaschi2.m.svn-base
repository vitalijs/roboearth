nCamaras = 7;
nPtos = 42;
nMed_pto = 2;
nParm_pto = 3;
nParm_cam = 6;
nParm_cal = 5;

medidas = nCamaras * nMed_pto * nPtos;
incognitas = (nCamaras-1)*nParm_cam + nPtos*nParm_pto + nParm_cal;
ndg = medidas - incognitas;
chi2inv(0.95,ndg)

