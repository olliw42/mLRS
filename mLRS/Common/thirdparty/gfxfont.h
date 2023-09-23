//*******************************************************
// This file is taken with tiny edits from the Adafruit-GFX-Library project
// github repo: https://github.com/adafruit/Adafruit-GFX-Library
// license: https://github.com/adafruit/Adafruit-GFX-Library/blob/master/license.txt
//*******************************************************

// Font structures for newer Adafruit_GFX (1.1 and later).
// Example fonts are included in 'Fonts' directory.
// To use a font in your Arduino sketch, #include the corresponding .h
// file and pass address of GFXfont struct to setFont().  Pass NULL to
// revert to 'classic' fixed-space bitmap font.

#ifndef _GFXFONT_H_
#define _GFXFONT_H_


#ifndef PROGMEM
  #define PROGMEM
#endif


typedef struct { // Data stored PER GLYPH
	uint16_t bitmapOffset;     // Pointer into GFXfont->bitmap
	uint8_t  width, height;    // Bitmap dimensions in pixels
	uint8_t  xAdvance;         // Distance to advance cursor (x axis)
	int8_t   xOffset, yOffset; // Dist from cursor pos to UL corner
} GFXglyph;


typedef struct { // Data stored for FONT AS A WHOLE
	uint8_t  *bitmap;      // Glyph bitmaps, concatenated
	GFXglyph *glyph;       // Glyph array
	uint8_t   first, last; // ASCII extents
	uint8_t   yAdvance;    // Newline distance (y axis)
} GFXfont;


#endif // _GFXFONT_H_
