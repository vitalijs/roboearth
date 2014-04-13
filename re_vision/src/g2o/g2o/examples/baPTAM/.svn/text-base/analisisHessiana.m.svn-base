load Hpp.m
load Hpl.m
load Hll.m

H = [Hpp Hpl
        Hpl' Hll];
        
vp = sqrt(svd(H));

[vp(1)  vp(end)  vp(1)/vp(end)]
bar(vp)