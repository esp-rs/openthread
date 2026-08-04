/**
 * Rudimentary C library implementation of `snprintf` and `vsnprintf`.
 *
 * `snprintf` takes a variable number of arguments and that isn't supported in
 * Rust. So, instead, here is a really basic `snprintf` implementation, which
 * is enough for Nordic Semiconductor's `libbsd` to link and run (at least for
 * the parts we've tested so far).
 *
 * Copyright (c) 42 Technology, 2019.
 * Licensed under the Blue Oak Model Licence 1.0.0
 */

/* ======================================================================== *
 *
 * System Includes
 *
 * ======================================================================== */
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>

/* ======================================================================== *
 *
 * Private Function Declarations
 *
 * ======================================================================== */

static void write_output(
   char out, char* restrict str, size_t max, size_t* written );

static void write_padding(
   char* restrict str, size_t size, size_t* written, size_t len, unsigned long width, unsigned long precision, bool zero_pad, bool is_negative );

static void write_right_padding(
   char* restrict str, size_t size, size_t* written, size_t len, unsigned long precision, bool is_negative, unsigned long width );

static char upcase(char c);

/* ======================================================================== *
 *
 * Macros
 *
 * ======================================================================== */

/**
 * The longest number we can print is:
 *
 * -9223372036854775808
 */
#define MAXIMUM_NUMBER_LENGTH 21

/* ======================================================================== *
 *
 * External Function Declarations
 *
 * ======================================================================== */

/**
 * This is provided by `itoa.rs`. It converts a signed number to a string in the
 * specified radix.
 */
extern int32_t itoa(int64_t i, char* s, size_t s_len, uint8_t radix);

/**
 * This is provided by `itoa.rs`. It converts an unsigned number to a string
 * in the specified radix.
 */
extern int32_t utoa(uint64_t i, char* s, size_t s_len, uint8_t radix);

/**
 * This is provided by `strtoul.rs`. It converts a string to a long.
 */
extern unsigned long int strtoul(const char* str, char** endptr, int base);

/* ======================================================================== *
 *
 * Public Function Definitions
 *
 * ======================================================================== */

/**
 * vsnprintf - string formatting with a `va_list` into a fixed sized buffer.
 *
 * Supports:
 *
 * - d/i (decimal signed integer)
 * - u (decimal unsigned integer)
 * - x (hexadecimal unsigned integer)
 * - c (char)
 * - s (null-terminated string)
 * - % (literal percent sign)
 * - qualifiers: l, ll, z, width (numeric or `*`), (non-string) precision,
 *   left-space-pad, zero-pad, `-` (left-justify / right-space-pad)
 *
 * Does not support:
 *
 * - p (none)
 * - p (pointer)
 * - o (octal)
 * - e (scientific notation with a lowercase e)
 * - E (scientific notation with a capital E)
 * - f (decimal floating point)
 * - g (the shorter of %e and %f)
 * - G (the shorter of %E and %f)
 * - qualifiers: L, +, etc
 *
 * @param str the output buffer to write to
 * @param size the size of the output buffer
 * @param fmt the format string
 * @param ap the variable number of arguments to render according to the
 *     format string
 * @return the number of characters written to `str`
 */
int vsnprintf(
   char* restrict str, size_t size, const char* fmt, va_list ap )
{
   size_t written = 0;
   bool is_escape = false;
   int is_long = 0;
   bool is_size_t = false;
   unsigned long precision = -1;
   unsigned long width = 0;
   bool zero_pad = false;
   bool left_pad = false;

   while ( *fmt )
   {
      if ( is_escape )
      {
         is_escape = false;
         switch ( *fmt )
         {
         case 'z':
            if ( is_long || is_size_t ) {
               // not supported
               return -1;
            }
            is_size_t = true;
            is_escape = true;
            break;
         case 'l':
            is_long++;
            if ( is_long >= 3 ) {
               // not supported
               return -1;
            }
            is_escape = true;
            break;
         case 'u':
            {
               char s[MAXIMUM_NUMBER_LENGTH] = { 0 };
               unsigned long long ll = 0;
               if ( is_size_t )
               {
                  // Render %zu
                  ll = va_arg( ap, size_t );
               }
               else if ( is_long == 2 )
               {
                  // Render %llu
                  ll = va_arg( ap, unsigned long long );
               }
               else if ( is_long == 1 )
               {
                  // Render %lu
                  ll = va_arg( ap, unsigned long );
               }
               else
               {
                  // Render %u
                  ll = va_arg( ap, unsigned int );
               }
               utoa( ll, s, sizeof(s), 10 );
               write_padding( str, size, &written, strlen(s), left_pad ? 0 : width, precision, left_pad ? false : zero_pad, false );
               for ( const char* p = s; *p != '\0'; p++ )
               {
                  write_output( *p, str, size, &written );
               }
               if ( left_pad )
               {
                  write_right_padding( str, size, &written, strlen(s), precision, false, width );
               }
            }
            break;
         case 'x':
         case 'X':
            // Render %x and %X
            {
               unsigned long long ll;
               char s[MAXIMUM_NUMBER_LENGTH] = { 0 };
               if ( is_size_t )
               {
                  ll = va_arg( ap, size_t );
               }
               else if ( is_long == 2 )
               {
                  ll = va_arg( ap, unsigned long long );
               }
               else if ( is_long == 1 )
               {
                  ll = va_arg( ap, unsigned long );
               }
               else
               {
                  ll = va_arg( ap, unsigned int );
               }
               utoa( ll, s, sizeof(s), 16 );
               write_padding( str, size, &written, strlen(s), left_pad ? 0 : width, precision, left_pad ? false : zero_pad, false );
               for (const char* p = s; *p != '\0'; p++)
               {
                  char output_char = (*fmt == 'X') ? upcase(*p) : *p;
                  write_output( output_char, str, size, &written );
               }
               if ( left_pad )
               {
                  write_right_padding( str, size, &written, strlen(s), precision, false, width );
               }
            }
            break;
         case 'i':
         case 'd':
            {
               char s[MAXIMUM_NUMBER_LENGTH] = { 0 };
               signed long long ll = 0;
               unsigned long long ull = 0;
               if ( is_long == 2 )
               {
                  // Render %lld
                  ll = va_arg( ap, signed long long );
               }
               else if ( is_long == 1 )
               {
                  // Render %ld
                  ll = va_arg( ap, signed long );
               }
               else
               {
                  // Render %d
                  ll = va_arg( ap, signed int );
               }
               bool is_negative = ll < 0;
               ull = is_negative ? -ll : ll;
               utoa( ull, s, sizeof(s), 10 );
               write_padding( str, size, &written, strlen(s), left_pad ? 0 : width, precision, left_pad ? false : zero_pad, is_negative );
               for ( const char* p = s; *p != '\0'; p++ )
               {
                  write_output( *p, str, size, &written );
               }
               if ( left_pad )
               {
                  write_right_padding( str, size, &written, strlen(s), precision, is_negative, width );
               }
            }
            break;
         case 'c':
            // Render %c
            {
               char c = (char) va_arg( ap, int );
               write_output( c, str, size, &written );
            }
            break;
         case 's':
            // Render %s
            {
               const char *s = va_arg( ap, const char* );
               unsigned long count = precision;
               
               size_t len = strlen(s);
               if (precision != (unsigned long)-1 && precision < len) {
                  len = precision;
               }
               if ( !left_pad )
               {
                  write_padding( str, size, &written, len, width, precision, false, false );
               }

               while (count > 0 && *s != '\0')
               {
                  write_output(*s, str, size, &written);

                  s++;
                  if (precision != (unsigned long)-1) {
                     count--;
                  }
               }
               if ( left_pad )
               {
                  // A string precision truncates (handled by `len` above); it
                  // never extends, so it plays no part in the padding length.
                  write_right_padding( str, size, &written, len, (unsigned long)-1, false, width );
               }
            }
            break;
         case '.':
            // Parse a precision specifier
            {
               // Next up is either a number or a '*' that signifies that the number is in the arguments list
               char next = *++fmt;

               if (next == '*')
               {
                  precision = va_arg( ap, int );
               }
               else
               {
                  precision = strtoul(fmt, (char**) &fmt, 10);
                  // Strtoul sets the fmt pointer to the char after the number,
                  // however the code expects the char before that.
                  fmt--;
               }

               is_escape = true;
            }
            break;
         case '-':
            // Left-justify: content first, space padding to the right (and,
            // per C, `-` overrides `0`).
            left_pad = true;
            is_escape = true;
            break;
         case '*':
            // Field width from the argument list; negative means
            // left-justify with the absolute value as the width.
            {
               int arg_width = va_arg( ap, int );
               if ( arg_width < 0 )
               {
                  left_pad = true;
                  arg_width = -arg_width;
               }
               width = (unsigned long) arg_width;
               is_escape = true;
            }
            break;
         case '0':
            // Parse zero padding specifier
            zero_pad = true;
            fmt++;
            /* fall through */
         case '1':
         case '2':
         case '3':
         case '4':
         case '5':
         case '6':
         case '7':
         case '8':
         case '9':
            // Parse padding specifier
            width = strtoul(fmt, (char**) &fmt, 10);
            // Strtoul sets the fmt pointer to the char after the number,
            // however the code expects the char before that.
            fmt--;
            is_escape = true;
            break;
         case '%':
            write_output( '%', str, size, &written );
            break;
         default:
            /* Ignore unknown format code */
            break;
         }
         fmt++;

         if (!is_escape) {
            // Reset precision if it hasn't just been assigned
            precision = -1;
         }
      }
      else
      {
         switch ( *fmt )
         {
         case '%':
            is_escape = true;
            is_long = 0;
            is_size_t = false;
            zero_pad  = false;
            left_pad = false;
            width = 0;
            precision = -1;
            break;
         default:
            write_output( *fmt, str, size, &written );
            break;
         }
         fmt++;
      }
   }

   /* Add a terminating null (but don't count it) */
   if ( written < size )
   {
      str[written] = '\0';
   }

   return (int) written;
}

/**
 * snprintf - string formatting with `...` into a fixed sized buffer.
 *
 * Grabs the var-args as a `va_list` and calls `vsnprintf`.
 *
 * @param str the output buffer to write to
 * @param size the size of the output buffer
 * @param fmt the format string
 * @param ... the variable number of arguments to render according to the
 *     format string
 * @return the number of characters written to `str`
 */
int snprintf( char* restrict str, size_t size, const char* restrict fmt, ... )
{
   va_list ap;
   va_start( ap, fmt );
   int result = vsnprintf( str, size, fmt, ap );
   va_end( ap );
   return result;
}

/* ======================================================================== *
 *
 * Private Function Definitions
 *
 * ======================================================================== */

/**
 * write_output - Add a character to a bounded length buffer.
 *
 * If the character doesn't fit in the buffer, it is dropped, but the counter
 * is still incremented.
 *
 * @param out the character to write
 * @param str the buffer to write to
 * @param written pass in the number of characters in the buffer; is increased
 *     by one regardless of whether we wrote to the buffer
 * @param size the total size of `str`
 */
static void write_output(
   char out, char* restrict str, size_t size, size_t* written )
{
   // Check if it fits
   if ( *written < size )
   {
      // Write the character
      str[*written] = out;
      *written = *written + 1;
   }
   else
   {
      *written = *written + 1;
   }
}

/**
 * write_padding - Write padding to the output buffer.
 * 
 * @param str the buffer to write to
 * @param size the total size of `str`
 * @param written pass in the number of characters in the buffer; is increased
 *    by one regardless of whether we wrote to the buffer
 * @param len the length of the string to write
 * @param width the total width of the padding
 * @param precision the precision of the padding
 * @param zero_pad whether to zero pad the string
 * @param is_negative whether the number is negative
 */
static void write_padding(char* restrict str, size_t size, size_t* written, size_t len, unsigned long width, unsigned long precision, bool zero_pad, bool is_negative) {
   if ( is_negative )
   {
      len++;
   }
   unsigned long pad_len = width > len ? width - len : 0;
   unsigned long zero_pad_len = 0;
   if ( precision != 0 && precision != (unsigned long)-1 )
   {
      if ( is_negative )
      {
         zero_pad_len = precision >= len ? precision - len + 1 : 0;
      }
      else
      {
         zero_pad_len = precision >= len ? precision - len : 0;
      }
   }
   else if ( zero_pad && precision == (unsigned long)-1 )
   {
      zero_pad_len = pad_len;
   }
   // Apply whitespace padding if needed
   pad_len = (zero_pad_len > pad_len) ? 0 : pad_len - zero_pad_len;
   for (unsigned long i = 0; i < pad_len; i++)
   {
      write_output( ' ', str, size, written );
   }
   // Apply zero padding if needed
   if (is_negative)
   {
      write_output( '-', str, size, written );
   }
   for (unsigned long i = 0; i < zero_pad_len; i++)
   {
      write_output( '0', str, size, written );
   }
}

/**
 * write_right_padding - Write the right-hand space padding of a
 * left-justified (`%-`) conversion.
 *
 * @param str the buffer to write to
 * @param size the total size of `str`
 * @param written pass in the number of characters in the buffer; increased
 *    for every padding character, written or dropped
 * @param len the length of the rendered content, excluding any sign and any
 *    zeros a numeric precision adds
 * @param precision the numeric precision (pass -1 for strings: a string
 *    precision truncates, it never extends)
 * @param is_negative whether a minus sign preceded the content
 * @param width the total field width to pad out to
 */
static void write_right_padding(
   char* restrict str, size_t size, size_t* written, size_t len, unsigned long precision, bool is_negative, unsigned long width )
{
   size_t content = len;
   if ( precision != (unsigned long)-1 && (size_t) precision > content )
   {
      content = precision;
   }
   if ( is_negative )
   {
      content++;
   }
   for ( size_t i = content; i < (size_t) width; i++ )
   {
      write_output( ' ', str, size, written );
   }
}

/**
 * Converts 'a'..'z' to 'A'..'Z', leaving all other characters unchanged.
 */
static char upcase(char c) {
   if (( c >= 'a' ) && ( c <= 'z' ))
   {
      return (c - 'a') + 'A';
   }
   else
   {
      return c;
   }
}

/* ======================================================================== *
 *
 * End of File
 *
 * ======================================================================== */