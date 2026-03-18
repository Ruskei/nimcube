import meshing
import sim
import c_api

proc NimMain() {.cdecl, importc.}
proc NimDestroyGlobals() {.cdecl, importc.}

proc library_init() {.cdecl, exportc, dynlib.} =
  NimMain()
  echo "Nim init"

proc library_deinit() {.cdecl, exportc, dynlib.} =
  echo "Nim deinit"
  deinit_worlds()
  NimDestroyGlobals()
