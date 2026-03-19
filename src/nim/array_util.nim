proc grown_capacity*(current_capacity, required_capacity: int): int =
  result =
    if current_capacity == 0:
      1
    else:
      current_capacity * 2
  if result < required_capacity:
    result = required_capacity

proc resize_shared_array*[T](
  data: var ptr UncheckedArray[T],
  old_capacity: int,
  new_capacity: int,
) =
  let old_size = Natural(old_capacity * sizeof(T))
  let new_size = Natural(new_capacity * sizeof(T))
  if data == nil:
    data = cast[ptr UncheckedArray[T]](allocShared0(new_size))
  else:
    data = cast[ptr UncheckedArray[T]](reallocShared0(data, old_size, new_size))
