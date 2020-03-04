source "nicp_post.m"
global n_points=1000;
global iterations=10;

Fixed=makeSamples(n_points);
T_good=inv(v2t([1,2,3,.1,.2,.3]'));
Moving=remapPoints(Fixed, T_good);


T=eye(4);
chi_evolution_direct=chiEvolution(Moving,
                                  Fixed,
                                  T,
                                  true,
                                  false,
                                  iterations);

T=eye(4);
chi_evolution_inverse=chiEvolution(Moving,
                                  Fixed,
                                  T,
                                  false,
                                  true,
                                  iterations);


T=eye(4);
chi_evolution_combo=chiEvolution(Moving,
                                 Fixed,
                                 T,
                                 true,
                                 true,
                                 iterations);

