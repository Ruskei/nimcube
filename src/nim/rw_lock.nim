import std/locks

type
  RwLock* = object
    mu: Lock
    can_read: Cond
    can_write: Cond
    readers: int
    waiting_writers: int
    writer_active: bool

proc init_rw_lock*(rw: var RwLock) =
  initLock(rw.mu)
  initCond(rw.can_read)
  initCond(rw.can_write)
  rw.readers = 0
  rw.waiting_writers = 0
  rw.writer_active = false

proc deinit_rw_lock*(rw: var RwLock) =
  deinitCond(rw.can_read)
  deinitCond(rw.can_write)
  deinitLock(rw.mu)

proc acquire_read*(rw: var RwLock) =
  acquire(rw.mu)
  try:
    while rw.writer_active or rw.waiting_writers > 0:
      wait(rw.can_read, rw.mu)
    inc rw.readers
  finally:
    release(rw.mu)

proc release_read*(rw: var RwLock) =
  acquire(rw.mu)
  try:
    dec rw.readers
    if rw.readers == 0 and rw.waiting_writers > 0:
      signal(rw.can_write)
  finally:
    release(rw.mu)

proc acquire_write*(rw: var RwLock) =
  acquire(rw.mu)
  try:
    inc rw.waiting_writers
    while rw.writer_active or rw.readers > 0:
      wait(rw.can_write, rw.mu)
    dec rw.waiting_writers
    rw.writer_active = true
  finally:
    release(rw.mu)

proc release_write*(rw: var RwLock) =
  acquire(rw.mu)
  try:
    rw.writer_active = false
    if rw.waiting_writers > 0:
      signal(rw.can_write)
    else:
      broadcast(rw.can_read)
  finally:
    release(rw.mu)

template with_read_lock*(rw: var RwLock; body: untyped) =
  acquire_read(rw)
  try:
    body
  finally:
    release_read(rw)

template with_write_lock*(rw: var RwLock; body: untyped) =
  acquire_write(rw)
  try:
    body
  finally:
    release_write(rw)
