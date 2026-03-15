import meshing

proc NimMain() {.cdecl, importc.}
proc NimDestroyGlobals() {.cdecl, importc.}

proc library_init() {.cdecl, exportc, dynlib.} =
  NimMain()
  echo "Nim init"

proc library_deinit() {.cdecl, exportc, dynlib.} =
  echo "Nim deinit"
  NimDestroyGlobals()
  GC_FullCollect()
