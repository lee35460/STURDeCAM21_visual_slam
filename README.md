# STURDeCAM21_visual_slam
```
mkdir build && cd build
cmake ..
make
./vo
```

# 서브모듈의 최신 커밋으로 업데이트하기:
```
cd [서브모듈 디렉토리]
git pull origin develop
```

# 메인 모듈에 서브모듈의 변경사항 커밋하기:
## 메인 모듈의 리포지토리로 돌아와서, 서브모듈의 변경사항을 메인 모듈의 리포지토리에 커밋해야 합니다.
```
cd ..
git add [서브모듈 디렉토리]
git commit -m "Update submodule to latest commit"
git push
```
