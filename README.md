# Global Router

## Build
```
mkdir build
cd build 
cmake ../ && cmake --build ./
```

## Run
```
cd ../bin

./ctr ../testcase/sample.in ../output/sample.out

./ctr ../testcase/test1.in ../output/test1.out

./ctr ../testcase/test2.in ../output/test2.out

./ctr ../testcase/test3.in ../output/test3.out

./ctr ../testcase/test4.in ../output/test4.out

./ctr ../testcase/test5.in ../output/test5.out


```

## Verify
```
python eval.py --input testcase/sample.in --output output/sample.out --plot true

python eval.py --input testcase/test1.in --output output/test1.out --fig results --plot true

python eval.py --input testcase/test2.in --output output/test2.out --fig results --plot true

python eval.py --input testcase/test3.in --output output/test3.out --fig results --plot true

python eval.py --input testcase/test4.in --output output/test4.out --fig results --plot true

python eval.py --input testcase/test5.in --output output/test5.out --fig results --plot true

```
