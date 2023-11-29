module YAML_Inp
use paraconf
use paraconf, only: YAML_tree => PC_tree_t
use NWTC_Base
use NWTC_Num
implicit none

private

interface
   function PC_parse_string_C(string) bind(C, name="PC_parse_string")
      use paraconf
      implicit none
      type(C_ptr), value :: string
      type(PC_tree_t) :: PC_parse_string_C
   end function PC_parse_string_C
end interface

interface YAML_Get
   module procedure YAML_GetListElement, YAML_GetSiKi, YAML_GetDbKi, &
      YAML_GetLogical, YAML_GetString, &
      YAML_GetIntKi, YAML_GetSiKiRank1, YAML_GetDbKiRank1
end interface

public :: YAML_ParsePath, YAML_DestroyTree, YAML_ListSize, YAML_Get, YAML_tree, YAML_HasKey

contains

subroutine YAML_ParsePath(path, tree, ErrStat, ErrMsg)
   character(*), intent(in)      :: path
   type(YAML_tree), intent(out)  :: tree
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg

   ErrStat = ErrID_None
   ErrMsg = ""

   call PC_errhandler(PC_NULL_HANDLER)

   call PC_parse_path(path, tree)
   if (PC_status(tree) /= PC_OK) then
      ErrStat = ErrID_Fatal
      call PC_errmsg(ErrMsg)
      return
   end if
end subroutine

subroutine YAML_ParseString(str, tree, ErrStat, ErrMsg)
   character(*), intent(in)      :: str
   type(YAML_tree), intent(out)  :: tree
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg

   integer :: i
   character(C_char), target :: C_str(len_trim(str) + 1)

   ErrStat = ErrID_None
   ErrMsg = ""

   do i = 1, len_trim(str)
      C_str(i) = str(i:i)
   end do
   C_str(len_trim(str) + 1) = C_NULL_CHAR

   call PC_errhandler(PC_NULL_HANDLER)

   tree = PC_parse_string_C(c_loc(C_str))

   if (PC_status(tree) /= PC_OK) then
      ErrStat = ErrID_Fatal
      call PC_errmsg(ErrMsg)
      return
   end if
end subroutine

subroutine YAML_DestroyTree(tree, ErrStat, ErrMsg)
   type(YAML_tree), intent(inout)   :: tree
   integer(IntKi), intent(out)      :: ErrStat
   character(*), intent(out)        :: ErrMsg
   integer(IntKi)                   :: status
   ErrStat = ErrID_None
   ErrMsg = ""
   call PC_tree_destroy(tree, status=status)
   if (status /= PC_OK) then
      ErrStat = ErrID_Fatal
      call PC_errmsg(ErrMsg)
      return
   end if
end subroutine

logical function YAML_HasKey(tree, key) result(exists)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   type(YAML_tree)               :: keyTree
   exists = PC_status(PC_get(tree, "."//key)) == PC_OK
end function

subroutine YAML_ListSize(tree, key, lsize, ErrStat, ErrMsg)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   integer(IntKi), intent(out)   :: lsize
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg
   type(YAML_tree)               :: keyTree
   integer(IntKi)                :: status
   keyTree = PC_get(tree, "."//key)
   if (PC_status(keyTree) == PC_OK) then
      call PC_len(keyTree, lsize, status=status)
      if (status /= PC_OK) then
         ErrStat = ErrID_Fatal
         call PC_errmsg(ErrMsg)
         return
      end if
   else
      ErrStat = ErrID_Fatal
      ErrMsg = "key "//key//" not found"
   end if
end subroutine

!-------------------------------------------------------------------------------
! Get routines
!-------------------------------------------------------------------------------

subroutine YAML_GetListElement(tree, key, ind, sub, ErrStat, ErrMsg)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   integer(IntKi), intent(in)    :: ind
   type(YAML_tree), intent(out)  :: sub
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg
   ErrStat = ErrID_None
   ErrMsg = ""
   sub = PC_get(tree, "."//key//"["//trim(Num2LStr(ind - 1))//"]")
   if (PC_status(sub) /= PC_OK) then
      ErrStat = ErrID_Fatal
      ErrMsg = "key "//key//" not found"
   end if
end subroutine

subroutine YAML_GetSiKi(tree, key, value, ErrStat, ErrMsg)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   real(SiKi), intent(out)       :: value
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg
   type(YAML_tree)               :: keyTree
   real(R8Ki)                    :: value_loc
   integer(IntKi)                :: status
   ErrStat = ErrID_None
   ErrMsg = ""
   keyTree = PC_get(tree, "."//key)
   if (PC_status(keyTree) == PC_OK) then
      call PC_double(keyTree, value_loc, status=status)
      if (status /= PC_OK) then
         ErrStat = ErrID_Fatal
         call PC_errmsg(ErrMsg)
         return
      end if
      value = real(value_loc, SiKi)
   else
      ErrStat = ErrID_Fatal
      ErrMsg = "key "//key//" not found"
   end if
end subroutine

subroutine YAML_GetSiKiRank1(tree, key, value, ErrStat, ErrMsg)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   real(SiKi), intent(inout)     :: value(:)
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg
   type(YAML_tree)               :: elemTree
   real(R8Ki)                    :: value_loc
   integer(IntKi)                :: i, status
   character(128)                :: elemKey
   ErrStat = ErrID_None
   ErrMsg = ""
   do i = 1, size(value)
      call YAML_GetListElement(tree, key, i, elemTree, ErrStat, ErrMsg)
      if (ErrStat /= ErrID_None) return
      if (PC_status(elemTree) == PC_OK) then
         call PC_double(elemTree, value_loc, status=status)
         if (status /= PC_OK) then
            ErrStat = ErrID_Fatal
            call PC_errmsg(ErrMsg)
            return
         end if
         value(i) = real(value_loc, SiKi)
      else
         ErrStat = ErrID_Fatal
         ErrMsg = "key "//elemKey//" not found"
      end if
   end do
end subroutine

subroutine YAML_GetDbKi(tree, key, value, ErrStat, ErrMsg)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   real(DbKi), intent(out)       :: value
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg
   type(YAML_tree)               :: keyTree
   real(R8Ki)                    :: value_loc
   integer(IntKi)                :: status
   ErrStat = ErrID_None
   ErrMsg = ""
   keyTree = PC_get(tree, "."//key)
   if (PC_status(keyTree) == PC_OK) then
      call PC_double(keyTree, value_loc, status=status)
      if (status /= PC_OK) then
         ErrStat = ErrID_Fatal
         call PC_errmsg(ErrMsg)
         return
      end if
      value = real(value_loc, DbKi)
   else
      ErrStat = ErrID_Fatal
      ErrMsg = "key "//key//" not found"
   end if
end subroutine

subroutine YAML_GetDbKiRank1(tree, key, value, ErrStat, ErrMsg)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   real(DbKi), intent(inout)     :: value(:)
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg
   type(YAML_tree)               :: elemTree
   real(R8Ki)                    :: value_loc
   integer(IntKi)                :: i, status
   character(128)                :: elemKey
   ErrStat = ErrID_None
   ErrMsg = ""
   do i = 1, size(value)
      call YAML_GetListElement(tree, key, i, elemTree, ErrStat, ErrMsg)
      if (ErrStat /= ErrID_None) return
      if (PC_status(elemTree) == PC_OK) then
         call PC_double(elemTree, value_loc, status=status)
         if (status /= PC_OK) then
            ErrStat = ErrID_Fatal
            call PC_errmsg(ErrMsg)
            return
         end if
         value(i) = real(value_loc, DbKi)
      else
         ErrStat = ErrID_Fatal
         ErrMsg = "key "//elemKey//" not found"
      end if
   end do
end subroutine

subroutine YAML_GetIntKi(tree, key, value, ErrStat, ErrMsg)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   integer(IntKi), intent(out)   :: value
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg
   type(YAML_tree)               :: keyTree
   integer(IntKi)                :: status
   ErrStat = ErrID_None
   ErrMsg = ""
   keyTree = PC_get(tree, "."//key)
   if (PC_status(keyTree) == PC_OK) then
      call PC_int(keyTree, value, status=status)
      if (status /= PC_OK) then
         ErrStat = ErrID_Fatal
         call PC_errmsg(ErrMsg)
         return
      end if
   else
      ErrStat = ErrID_Fatal
      ErrMsg = "key "//key//" not found"
   end if
end subroutine

subroutine YAML_GetLogical(tree, key, value, ErrStat, ErrMsg)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   logical, intent(out)          :: value
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg
   type(YAML_tree)               :: keyTree
   integer(IntKi)                :: status
   ErrStat = ErrID_None
   ErrMsg = ""
   keyTree = PC_get(tree, "."//key)
   if (PC_status(keyTree) == PC_OK) then
      call PC_log(keyTree, value, status=status)
      if (status /= PC_OK) then
         ErrStat = ErrID_Fatal
         call PC_errmsg(ErrMsg)
         return
      end if
   else
      ErrStat = ErrID_Fatal
      ErrMsg = "key "//key//" not found"
   end if
end subroutine

subroutine YAML_GetString(tree, key, value, ErrStat, ErrMsg)
   type(YAML_tree), intent(in)   :: tree
   character(*), intent(in)      :: key
   character(*), intent(out)     :: value
   integer(IntKi), intent(out)   :: ErrStat
   character(*), intent(out)     :: ErrMsg
   type(YAML_tree)               :: keyTree
   integer(IntKi)                :: status
   ErrStat = ErrID_None
   ErrMsg = ""
   keyTree = PC_get(tree, "."//key)
   if (PC_status(keyTree) == PC_OK) then
      call PC_string(keyTree, value, status=status)
      if (status /= PC_OK) then
         ErrStat = ErrID_Fatal
         call PC_errmsg(ErrMsg)
         return
      end if
   else
      ErrStat = ErrID_Fatal
      ErrMsg = "key "//key//" not found"
   end if
end subroutine

end module YAML_Inp
