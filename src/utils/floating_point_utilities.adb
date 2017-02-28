------------------------------------------------------------------------------
--                                                                          --
--                    Copyright (C) 2016, AdaCore                           --
--                                                                          --
--  Redistribution and use in source and binary forms, with or without      --
--  modification, are permitted provided that the following conditions are  --
--  met:                                                                    --
--     1. Redistributions of source code must retain the above copyright    --
--        notice, this list of conditions and the following disclaimer.     --
--     2. Redistributions in binary form must reproduce the above copyright --
--        notice, this list of conditions and the following disclaimer in   --
--        the documentation and/or other materials provided with the        --
--        distribution.                                                     --
--     3. Neither the name of STMicroelectronics nor the names of its       --
--        contributors may be used to endorse or promote products derived   --
--        from this software without specific prior written permission.     --
--                                                                          --
--   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    --
--   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      --
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  --
--   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   --
--   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, --
--   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       --
--   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  --
--   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  --
--   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    --
--   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  --
--   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   --
--                                                                          --
------------------------------------------------------------------------------

package body Floating_Point_Utilities is

   Value : constant array (Character range '0' .. '9') of Real :=
     (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

   function Stripped (Input : String) return String;

   procedure Skip_Blanks (S : in String; First : out Natural);

   function Image (Whole_Part : String; Fraction : Real) return String;

   --------------
   -- Stripped --
   --------------

   function Stripped (Input : String) return String is
      First : Integer range Input'Range := Input'Last;
      Last  : Integer range Input'Range := Input'First;
   begin
      for I in Input'Range loop
         if Input (I) /= ' ' then
            First := I;
            exit;
         end if;
      end loop;
      for I in reverse Input'Range loop
         if Input (I) /= ' ' then
            Last := I;
            exit;
         end if;
      end loop;
      return Input (First .. Last);
   end Stripped;

   -----------------
   -- Skip_Blanks --
   -----------------

   procedure Skip_Blanks (S : in String; First : out Natural) is
   begin
      First := S'First;
      for I in S'Range loop
         if S (I) /= ' ' then
            First := I;
            exit;
         end if;
      end loop;
   end Skip_Blanks;

   ---------------
   -- Are_Equal --
   ---------------

   function Are_Equal (Left, Right : Real) return Boolean is
   begin
      return (abs (Left - Right) < Real'Small);
   end Are_Equal;

   -------------
   -- Is_Zero --
   -------------

   function Is_Zero (Left : Real) return Boolean is
   begin
      return (abs (Left) < Real'Small);
   end Is_Zero;

   ---------------
   -- Truncated --
   ---------------

   function Truncated (Input : Real) return Integer is
      Result : Integer := Integer (Input); -- rounds up or down
   begin
      if Input < 0.0 then
         if Input > Real (Result) then
            Result := Result + 1;
         end if;
      else
         if Input < Real (Result) then
            Result := Result - 1;
         end if;
      end if;
      return Result;
   exception
      when others =>
         raise Unknown_Error;
   end Truncated;

   ---------------------
   -- Fractional_Part --
   ---------------------

   function Fractional_Part (Input : Real) return Real is
   begin
      return abs (Input - Real (Truncated (Input)));
   exception
      when others =>
         raise Unknown_Error;
   end Fractional_Part;

   ----------------------
   -- Fractional_Image --
   ----------------------

   function Fractional_Image (Input : Real; Length : Natural) return String is
      Number : constant Real := Input * 10.0;
   begin
      if Length = 0 then
         return "";
      else
         return Stripped (Integer'Image (Truncated (Number))) &
           Fractional_Image (Fractional_Part (Number), Length - 1);
      end if;
   exception
      when others =>
         raise Unknown_Error;
   end Fractional_Image;

   -----------
   -- Image --
   -----------

   function Image (Whole_Part : String; Fraction : Real) return String is
      Significant_Digits : constant Natural := Real'Digits - Whole_Part'Length + 1;
   begin
      if Significant_Digits > 0 then
         return Whole_Part &
           '.' &
           Fractional_Image (Fraction, Significant_Digits);
      else
         return Whole_Part & ".0";
      end if;
   end Image;

   --------------
   -- Image_of --
   --------------

   function Image_of (Input : Real; Leading_Blank : Boolean := False) return String is
   begin
      if Input < 0.0 then
         return '-' &
           Image
             (Whole_Part => Stripped (Integer'Image (Truncated (abs (Input)))),
              Fraction   => Fractional_Part (Input));
      else
         return (if Leading_Blank then " " else "") &
           Image
             (Whole_Part => Stripped (Integer'Image (Truncated (abs (Input)))),
              Fraction   => Fractional_Part (Input));
      end if;
   exception
      when others =>
         raise Unknown_Error;
   end Image_of;

   --------------
   -- Value_of --
   --------------

   function Value_of (Input : String) return Real is
      Power         : Real    := 0.0;
      Total         : Real    := 0.0;
      Decimal_Point : Natural := 0;
      First         : Natural := 0;
      Negative      : Boolean := False;
   begin
      if Input'Length < 3 then -- form must be at least "X.X"
         raise Format_Error;
      end if;
      Skip_Blanks (Input, First);
      if Input (First) = '-' then
         Negative := True;
         First    := First + 1;
      end if;
      for I in First .. Input'Last loop
         if Input (I) = '.' then
            Decimal_Point := I;
            exit;
         end if;
      end loop;
      if Decimal_Point = 0 then -- never found it in Input
         raise Format_Error;
      end if;
      Power := 1.0;
      for I in reverse First .. (Decimal_Point - 1) loop
         Total := Total + (Value (Input (I)) * Power);
         Power := Power * 10.0;
      end loop;
      Power := 10.0;
      for I in (Decimal_Point + 1) .. Input'Last loop
         Total := Total + (Value (Input (I)) / Power);
         Power := Power * 10.0;
      end loop;
      if Negative then
         return -Total;
      else
         return Total;
      end if;
   exception
      when Constraint_Error => -- presumably (?!) caused by bad input form
         raise Format_Error;
      when others =>
         raise Unknown_Error;
   end Value_of;

end Floating_Point_Utilities;
