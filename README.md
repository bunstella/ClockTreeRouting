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

./cts --input ../testcase/sample.in --output ../output/sample.out

./cts --input ../testcase/test1.in --output ../output/test1.out

./cts --input ../testcase/test2.in --output ../output/test2.out

./cts --input ../testcase/test3.in --output ../output/test3.out

./cts --input ../testcase/test4.in --output ../output/test4.out

./cts --input ../testcase/test5.in --output ../output/test5.out
```

# Configurations
| Parameter               | Description                          |
-----------------------------------------------------------------|
| verify                  | Evaluate quality                     |
| plot                    | Plot result                          |

## Verify
```
python eval.py --input testcase/sample.in --output output/sample.out

python eval.py --input testcase/test1.in --output output/test1.out

python eval.py --input testcase/test2.in --output output/test2.out

python eval.py --input testcase/test3.in --output output/test3.out

python eval.py --input testcase/test4.in --output output/test4.out

python eval.py --input testcase/test5.in --output output/test5.out

```