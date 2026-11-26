function ls = nav_lat_init()
ls.st = pid_init(0);
ls.pf = pid_prefilt_init(0);
ls.act = 0;
ls.eyp = 0; ls.c = 0;
end
