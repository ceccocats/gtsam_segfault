# gtsam segfault error

## build deps
- eigen: https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
- gtsam: https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.tar.gz
- build gtsam with this flags: -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF

## build app
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make 
gdb ./gtsam_opt
```

## output
- app will randomly raise a SEGFAULT after some time, like this:
<details>
  <summary>output</summary>

```
Starting program: /home/cecco/ws/rtls/gtsam_segfault/build/gtsam_opt 
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/lib/x86_64-linux-gnu/libthread_db.so.1".
load graph: ../data/graph_101.dat
graph size: 38432 estimate size: 4436
optimize
[New Thread 0x7ffff6db6700 (LWP 418872)]
[New Thread 0x7ffff65b4700 (LWP 418874)]
[New Thread 0x7ffff69b5700 (LWP 418873)]
[New Thread 0x7ffff5db2700 (LWP 418876)]
[New Thread 0x7ffff59b1700 (LWP 418877)]
[New Thread 0x7ffff61b3700 (LWP 418875)]
[New Thread 0x7ffff55b0700 (LWP 418878)]
[New Thread 0x7ffff49ad700 (LWP 418880)]
[New Thread 0x7ffff4dae700 (LWP 418881)]
[New Thread 0x7ffff51af700 (LWP 418879)]
[New Thread 0x7ffff45ac700 (LWP 418882)]
optimize
optimize
optimize
optimize
load graph: ../data/graph_126.dat
graph size: 47120 estimate size: 5464
optimize
optimize
optimize
optimize
optimize
load graph: ../data/graph_151.dat
graph size: 54800 estimate size: 6481
optimize
optimize
optimize
optimize
optimize
load graph: ../data/graph_26.dat
graph size: 9015 estimate size: 1122
optimize
optimize
optimize
optimize
optimize
load graph: ../data/graph_51.dat
graph size: 19956 estimate size: 2362
optimize
optimize
[New Thread 0x7fffc4dfb700 (LWP 419000)]
[New Thread 0x7fffbc9fa700 (LWP 419001)]
optimize
optimize
optimize
load graph: ../data/graph_76.dat
graph size: 28699 estimate size: 3289
optimize
optimize
optimize
optimize
optimize
load graph: ../data/graph_101.dat
graph size: 38432 estimate size: 4436
optimize
optimize
optimize
optimize
optimize
load graph: ../data/graph_126.dat
graph size: 47120 estimate size: 5464
optimize
optimize
optimize
optimize
terminate called after throwing an instance of 'std::out_of_range'
  what():  map::at

Thread 1 "gtsam_opt" received signal SIGABRT, Aborted.
__GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
50      ../sysdeps/unix/sysv/linux/raise.c: No such file or directory.
(gdb) bt
#0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
#1  0x00007ffff75af859 in __GI_abort () at abort.c:79
#2  0x00007ffff7835911 in ?? () from /lib/x86_64-linux-gnu/libstdc++.so.6
#3  0x00007ffff784138c in ?? () from /lib/x86_64-linux-gnu/libstdc++.so.6
#4  0x00007ffff78413f7 in std::terminate() () from /lib/x86_64-linux-gnu/libstdc++.so.6
#5  0x00007ffff78416fd in __cxa_rethrow () from /lib/x86_64-linux-gnu/libstdc++.so.6
#6  0x00007ffff7d2e60d in gtsam::VectorValues gtsam::internal::linearAlgorithms::optimizeBayesTree<gtsam::GaussianBayesTree>(gtsam::GaussianBayesTree const&)
    () from /home/cecco/ws/tk_stuff/tkDeps/install/lib/libgtsam.so.4
#7  0x00007ffff7d1c632 in gtsam::GaussianBayesTree::optimize() const () from /home/cecco/ws/tk_stuff/tkDeps/install/lib/libgtsam.so.4
#8  0x00007ffff7d46a62 in gtsam::GaussianFactorGraph::optimize(gtsam::Ordering const&, std::function<std::pair<boost::shared_ptr<gtsam::GaussianConditional>, boost::shared_ptr<gtsam::GaussianFactor> > (gtsam::GaussianFactorGraph const&, gtsam::Ordering const&)> const&) const ()
   from /home/cecco/ws/tk_stuff/tkDeps/install/lib/libgtsam.so.4
#9  0x00007ffff7e2fd6b in gtsam::NonlinearOptimizer::solve(gtsam::GaussianFactorGraph const&, gtsam::NonlinearOptimizerParams const&) const ()
   from /home/cecco/ws/tk_stuff/tkDeps/install/lib/libgtsam.so.4
#10 0x00007ffff7e12851 in gtsam::LevenbergMarquardtOptimizer::tryLambda(gtsam::GaussianFactorGraph const&, gtsam::VectorValues const&) ()
   from /home/cecco/ws/tk_stuff/tkDeps/install/lib/libgtsam.so.4
#11 0x00007ffff7e13818 in gtsam::LevenbergMarquardtOptimizer::iterate() () from /home/cecco/ws/tk_stuff/tkDeps/install/lib/libgtsam.so.4
#12 0x00007ffff7e2f078 in gtsam::NonlinearOptimizer::defaultOptimize() () from /home/cecco/ws/tk_stuff/tkDeps/install/lib/libgtsam.so.4


#13 0x00005555555b235a in gtsam::NonlinearOptimizer::optimize (this=0x7fffffffd1e0)
    at /home/cecco/ws/tk_stuff/tkDeps/install/include/gtsam/nonlinear/NonlinearOptimizer.h:98
#14 0x00005555555aaa26 in basalt::GraphOpt::optimize (this=0x7fffffffd390, maxIterations=1) at /home/cecco/ws/rtls/gtsam_segfault/GraphOpt.cpp:132
--Type <RET> for more, q to quit, c to continue without paging--
#15 0x00005555555a1baf in main (argc=1, argv=0x7fffffffd7d8) at /home/cecco/ws/rtls/gtsam_segfault/gtsam_opt.cpp:18
(gdb) 
```
  
</details>