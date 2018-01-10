/************************************************************************************/
/* Copyright (C) 2009-2010, Luis Alvarez <lalvarez@dis.ulpgc.es> */
/* FUNCIONES DE LECTURA Y ESCRITURA DE  IMAGENES EN FORMATO bmp */

#ifndef AMI_BMP_H
#define AMI_BMP_H

typedef struct {
   unsigned short int type;                 /* Magic identifier            */
   unsigned int size;                       /* File size in bytes          */
   unsigned short int reserved1, reserved2;
   unsigned int offset;                     /* Offset to image data, bytes */
} HEADER;
typedef struct {
   unsigned int size;               /* Header size in bytes      */
   int width,height;                /* Width and height of image */
   unsigned short int planes;       /* Number of colour planes   */
   unsigned short int bits;         /* Bits per pixel            */
   unsigned int compression;        /* Compression type          */
   unsigned int imagesize;          /* Image size in bytes       */
   int xresolution,yresolution;     /* Pixels per meter          */
   unsigned int ncolours;           /* Number of colours         */
   unsigned int importantcolours;   /* Important colours         */
} INFOHEADER;


/* FUNCTION TO WRITE A 24 bits COLOR BMP FORMAT IMAGE  (RETURN 0 IF FINISH PROPERLY -1 OTHERWISE) */
int ami_write_bmp(char name[200], unsigned char *red, unsigned char *green, unsigned char *blue, int width, int height);
/* FUNCTION TO READ A 24 bits COLOR BMP FORMAT IMAGE  (RETURN 0 IF FINISH PROPERLY -1 OTHERWISE) */
int ami_read_bmp(char name[200], unsigned char **red, unsigned char **green, unsigned char **blue, int *width, int *height);

#endif
