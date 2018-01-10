//Copyright (C) 2009-2010, Luis Alvarez <lalvarez@dis.ulpgc.es>
#include <stdio.h>
#include <stdlib.h>
#include "ami_bmp.h"


/* FUNCTION TO READ A 24 bits COLOR BMP FORMAT IMAGE  (RETURN 0 IF FINISH PROPERLY -1 OTHERWISE) */
int ami_read_bmp(char name[200], unsigned char **red, unsigned char **green, unsigned char **blue, int *width, int *height)
{
   int i,j;//,k;
   //unsigned char r,g,b;
   HEADER header;
   INFOHEADER infoheader;
   FILE *fptr;
   long size,cont; //m,
   unsigned char *image;

   /* Open file */
   if ((fptr = fopen(name,"rb")) == NULL) {
      fprintf(stderr,"Unable to open BMP file \"%s\"\n",name);
      return(-1);
   }

   /* Read and check the header */
   if (fread(&header.type,sizeof(unsigned short),1,fptr) != 1) {
       fprintf(stderr,"Image read failed\n");
       return(-1);
   }

   /* Read and check the header */
   fprintf(stderr,"ID is: %d, should be %d\n",header.type,'M'*256+'B');
   if (header.type != (int) 'M'*256+'B') {
     fprintf(stderr,"Sorry, the file is not a bmp image file (magic number not correct)\n");
     return(-1);
   }

   if (fread(&header.size,sizeof(unsigned int),1,fptr) != 1) {
               fprintf(stderr,"Image read failed\n");
               return(-1);
            }

   fprintf(stderr,"File size is %d bytes\n",header.size);
   if (fread(&header.reserved1,sizeof(unsigned short),1,fptr) != 1) {
               fprintf(stderr,"Image read failed\n");
               return(-1);
            }

   if (fread(&header.reserved2,sizeof(unsigned short),1,fptr) != 1) {
               fprintf(stderr,"Image read failed\n");
               return(-1);
   }
   fprintf(stderr,"header.reserved1 = %d \n",header.reserved1);
   if (fread(&header.offset,sizeof(unsigned int),1,fptr) != 1) {
               fprintf(stderr,"Image read failed\n");
               return(-1);
    }
    fprintf(stderr,"header.reserved2 = %d \n",header.reserved2);
    fprintf(stderr,"Offset to image data is %d bytes\n",header.offset);

   /* Read and check the information header */
   if (fread(&infoheader,sizeof(INFOHEADER),1,fptr) != 1) {
      fprintf(stderr,"Failed to read BMP info header\n");
      return(-1);
   }
   fprintf(stderr,"Header size = %d \n",infoheader.size);
   fprintf(stderr,"Image size = %d x %d\n",infoheader.width,infoheader.height);
   fprintf(stderr,"Number of colour planes is %d\n",infoheader.planes);
   fprintf(stderr,"Bits per pixel is %d\n",infoheader.bits);
   fprintf(stderr,"Compression type is %d\n",infoheader.compression);
   fprintf(stderr,"Infoheader.imagesize is %d\n",infoheader.imagesize);
   fprintf(stderr,"Infoheader.xresolution=%d infoheader.yresolution=%d \n",infoheader.xresolution,infoheader.yresolution);
   fprintf(stderr,"Number of colours is %d\n",infoheader.ncolours);
   fprintf(stderr,"Number of required colours is %d\n",
      infoheader.importantcolours);

   /* WE CHECK THE 24 bits */
   if(infoheader.bits!=24){
	   fprintf(stderr,"Sorry, function only implemented for 24 bits colour format\n");
       return(-1);
   }
   /* WE CHECK COMPRESSION */
   if(infoheader.compression!=0){
	   fprintf(stderr,"Sorry, function only implemented image without compression \n");
       return(-1);
   }
   /* WE CHECK THE 24 bits */
   if(infoheader.ncolours!=0){
	   fprintf(stderr,"Sorry, function only implemented for 24 bits colour format\n");
       return(-1);
   }
   /* WE CHECK THE 24 bits */
   if(infoheader.importantcolours!=0){
	   fprintf(stderr,"Sorry, function only implemented for 24 bits colour format\n");
       return(-1);
   }


   /* WE FIT DIMENSIONS AND ALLOCATE MEMORY FOR THE IMAGE */
   *width=infoheader.width; *height=infoheader.height;
   //size = ((*width)%4) ? ((*width)+4-((*width)%4))*(*height) : (*width)*(*height);
   size = (3*(*width) + (*width)%4)*(*height);
   *red=(unsigned char*) malloc(size*sizeof(unsigned char));
   *green=(unsigned char*) malloc(size*sizeof(unsigned char));
   *blue=(unsigned char*) malloc(size*sizeof(unsigned char));

  /* WE STORE THE IMAGE INFORMATION IN image */
   image=(unsigned char*)malloc(size*sizeof(unsigned char));

   /* We read the image */
   if (fread(image,size,1,fptr) != 1) {
     fprintf(stderr,"Image read failed\n");
     free(image);
     return(-1);
   }

   cont=0;
   for(i=0; i<(*height); i++){
     for(j=0; j<(*width); j++){
       (*blue)[i*(*width) + j]=image[cont++]; (*green)[i*(*width) + j]=image[cont++]; (*red)[i*(*width) + j]=image[cont++];
     }
     // Ignorar bytes rellenados con 0 para tamaño de fila múltiplo de 4 (considerando los 3 bytes RGB)
     cont+=(*width)%4;
   }

   fclose(fptr);
   free(image);
   return(0);
}


/* FUNCTION TO WRITE A 24 bits COLOR BMP FORMAT IMAGE  (RETURN 0 IF FINISH PROPERLY -1 OTHERWISE) */
int ami_write_bmp(char name[200], unsigned char *red, unsigned char *green, unsigned char *blue, int width, int height)
{
   int i,j,k;
   //unsigned char r,g,b;
   HEADER header;
   INFOHEADER infoheader;
   FILE *fptr;
   long cont,size;//,m;
   unsigned char *image;

   /* Open file */
   if ((fptr = fopen(name,"wb")) == NULL) {
      fprintf(stderr,"Unable to open BMP file \"%s\"\n",name);
      return(-1);
   }

   /* Header Definition */
   header.type = (unsigned short int) 'M'*256+'B';
   header.size = (unsigned int) 3*width*height+54;
   header.reserved1 = 0;
   header.reserved2 = 0;
   header.offset = (unsigned int) 54;


  /* We write the header */
   if (fwrite(&header.type,sizeof(unsigned short),1,fptr) != 1) {
      fprintf(stderr,"Failed to write BMP info header\n");
      return(-1);
   }
   if (fwrite(&header.size,sizeof(unsigned int),1,fptr) != 1) {
      fprintf(stderr,"Failed to write BMP info header\n");
      return(-1);
   }
   if (fwrite(&header.reserved1,sizeof(unsigned short),1,fptr) != 1) {
      fprintf(stderr,"Failed to write BMP info header\n");
      return(-1);
   }
   if (fwrite(&header.reserved2,sizeof(unsigned short),1,fptr) != 1) {
      fprintf(stderr,"Failed to write BMP info header\n");
      return(-1);
   }
   if (fwrite(&header.offset,sizeof(unsigned int),1,fptr) != 1) {
      fprintf(stderr,"Failed to write BMP info header\n");
      return(-1);
   }


   /* INFOHEADER DEFINITION */
   infoheader.size = 40;
   infoheader.width = width;//(width%4) ? width+4-(width%4) : width;
   infoheader.height = height;
   infoheader.planes = (unsigned short int) 1;
   infoheader.bits = (unsigned short int) 24;
   infoheader.compression = (unsigned int) 0;
   infoheader.imagesize = (3*width + width%4)*height;//(width%4) ? 3*(width+4-(width%4))*height : 3*width*height;
   infoheader.xresolution=2834;
   infoheader.yresolution=2834;
   infoheader.ncolours = (unsigned int) 0 ;
   infoheader.importantcolours = (unsigned int) 0;



   /* We write the infoheaderinformation */
   if (fwrite(&infoheader,sizeof(INFOHEADER),1,fptr) != 1) {
      fprintf(stderr,"Failed to write BMP info header\n");
      return(-1);
   }

   size = (3*width + width%4)*height;

   /* WE STORE THE IMAGE INFORMATION IN image */
   image=(unsigned char*)malloc(size*sizeof(unsigned char));
   //printf("size*3=%d\n",size*3);
   cont=0;
   for(i=0; i<height; i++){
     for(j=0; j<width; j++){
       image[cont++]=blue[i*width + j]; image[cont++]=green[i*width + j]; image[cont++]=red[i*width + j];
     }
     // Rellenar bytes con 0 para obtener tamaño de fila múltiplo de 4 (considerando los 3 bytes RGB)
     for(k=0; k<width%4; k++) image[cont++]=0;
   }

   /* We write the image */
  if (fwrite(image,size,1,fptr) != 1) {
     fprintf(stderr,"Image write failed\n");
     free(image);
     return(-1);
   }

   fclose(fptr);
   free(image);
   return(0);
}
