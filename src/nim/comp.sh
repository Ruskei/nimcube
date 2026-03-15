nim c --cc:clang \
  --cpu:arm64 \
  --os:linux \
  --app:lib \
  --passc:"-fsanitize=address -mllvm -asan-force-dynamic-shadow=1 -fno-omit-frame-pointer -O1 -g" \
  --passl:"-fsanitize=address" \
  main.nim
