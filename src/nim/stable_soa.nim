import std/macros
import physics_math

const soa_slot_invalid* = -1

type SoaFieldSpec = tuple[field_name: NimNode, field_type: NimNode]

proc exported(name: NimNode): NimNode =
  nnkPostfix.newTree(ident"*", name)

proc seq_type(inner: NimNode): NimNode =
  nnkBracketExpr.newTree(ident"seq", inner)

proc ident_defs(name, typ: NimNode): NimNode =
  nnkIdentDefs.newTree(name, typ, newEmptyNode())

proc parse_soa_fields(body: NimNode): seq[SoaFieldSpec] =
  for node in body:
    if node.kind != nnkCall:
      error("expected `field: Type` got: " & node.repr, node)
    result.add (field_name: node[0], field_type: node[1])

proc build_soa_type_section(name, handle_name: NimNode, fields: openArray[SoaFieldSpec]): NimNode =
  let rec_list = newNimNode(nnkRecList)

  for field in fields:
    rec_list.add ident_defs(exported(field.field_name), seq_type(field.field_type))

  for internal_name in ["dense_to_slot", "slot_to_dense", "generation", "free_slots"]:
    rec_list.add ident_defs(ident(internal_name), seq_type(ident"int"))

  result = nnkTypeSection.newTree(
    nnkTypeDef.newTree(
      exported(name),
      newEmptyNode(),
      nnkRefTy.newTree(
        nnkObjectTy.newTree(newEmptyNode(), newEmptyNode(), rec_list)
      )
    ),
    nnkTypeDef.newTree(
      exported(handle_name),
      newEmptyNode(),
      nnkObjectTy.newTree(
        newEmptyNode(),
        newEmptyNode(),
        nnkRecList.newTree(
          ident_defs(exported(ident"slot"), ident"int"),
          ident_defs(exported(ident"generation"), ident"int"),
        )
      )
    )
  )

proc build_add_proc(name, handle_name: NimNode, fields: openArray[SoaFieldSpec]): NimNode =
  let slot = ident"slot"
  let dense = ident"dense"
  let params = nnkFormalParams.newTree(
    handle_name,
    ident_defs(ident"data", name)
  )
  let body = quote do:
    var `slot`: int
    if data.free_slots.len > 0:
       `slot` = data.free_slots.pop()
    else:
      `slot` = data.generation.len
      data.generation.add 0
      data.slot_to_dense.add soa_slot_invalid
    let `dense` = data.dense_to_slot.len

  for field in fields:
    let field_name = field.field_name
    let field_type = field.field_type
    params.add ident_defs(field_name, field_type)
    body.add quote do:
      data.`field_name`.add `field_name`

  body.add quote do:
    data.dense_to_slot.add `slot`
    data.slot_to_dense[`slot`] = `dense`
    result = `handle_name`(slot: `slot`, generation: data.generation[`slot`])

  result = nnkProcDef.newTree(
    exported(ident"add"),
    newEmptyNode(),
    newEmptyNode(),
    params,
    newEmptyNode(),
    newEmptyNode(),
    body
  )

proc build_is_valid_proc(name, handle_name: NimNode): NimNode =
  quote do:
    proc is_valid*(data: `name`, handle: `handle_name`): bool =
      return
        handle.slot >= 0 and
        handle.slot < data.generation.len and
        data.generation[handle.slot] == handle.generation and
        data.slot_to_dense[handle.slot] != soa_slot_invalid

proc build_remove_proc(name, handle_name: NimNode, fields: openArray[SoaFieldSpec]): NimNode =
  let handle = ident"handle"
  let slot = ident"slot"
  let dense = ident"dense"
  let last_dense = ident"last_dense"
  let moved_slot = ident"moved_slot"
  let body = newStmtList()

  body.add quote do:
    if not data.is_valid(`handle`):
      return false
  body.add quote do:
    let `slot` = `handle`.slot
  body.add quote do:
    let `dense` = data.slot_to_dense[`slot`]
  body.add quote do:
    let `last_dense` = data.dense_to_slot.len - 1

  let swap_body = newStmtList(
    quote do:
      let `moved_slot` = data.dense_to_slot[`last_dense`]
  )
  for field in fields:
    let field_name = field.field_name
    swap_body.add quote do:
      data.`field_name`[`dense`] = data.`field_name`[`last_dense`]

  swap_body.add quote do:
    data.dense_to_slot[`dense`] = `moved_slot`
    data.slot_to_dense[`moved_slot`] = `dense`

  body.add nnkIfStmt.newTree(
    nnkElifBranch.newTree(
      quote do:
        `dense` != `last_dense`
      ,
      swap_body
    )
  )

  for field in fields:
    let field_name = field.field_name
    body.add quote do:
      data.`field_name`.setLen `last_dense`

  body.add quote do:
    data.dense_to_slot.setLen `last_dense`
    data.slot_to_dense[`slot`] = soa_slot_invalid
    inc data.generation[`slot`]
    data.free_slots.add `slot`
    result = true

  result = nnkProcDef.newTree(
    exported(ident"remove"),
    newEmptyNode(),
    newEmptyNode(),
    nnkFormalParams.newTree(
      ident"bool",
      ident_defs(ident"data", name),
      ident_defs(handle, handle_name),
    ),
    newEmptyNode(),
    newEmptyNode(),
    body
  )

proc build_field_accessor(field_name, field_type, name, handle_name: NimNode): NimNode =
  quote do:
    proc `field_name`*(data: `name`, handle: `handle_name`): `field_type` =
      result = data.`field_name`[data.slot_to_dense[handle.slot]]

macro declare_stable_soa_type*(name: untyped, body: untyped): untyped =
  let handle_name = ident($name & "Handle")
  let fields = parse_soa_fields(body)

  result = newStmtList(
    build_soa_type_section(name, handle_name, fields),
    build_add_proc(name, handle_name, fields),
    build_is_valid_proc(name, handle_name),
    build_remove_proc(name, handle_name, fields),
  )

  for field in fields:
    result.add build_field_accessor(field.field_name, field.field_type, name, handle_name)

  # echo result.repr

# declare_stable_soa_type InternalData:
#   pos: D3
#   ω: F3
