#include <stdio.h>
#include <stdlib.h>
#include "global.h"

/*
   Write the current view to a file
   The multiple fputc()s can be replaced with
      fwrite(image,width*height*3,1,fptr);
   If the memory pixel order is the same as the destination file format.
*/
int stereo = 0;
static int counter = 0;

/* This supports animation sequences */

int WindowDump(void) {

    counter += 1;
    if(counter % 10 != 0)
        return -1;
    int i, j;
    FILE *fptr;
    unsigned char *image;

    /* Allocate our buffer for the image */
    if ((image = (unsigned char *) malloc(3 * width * height * sizeof(char))) == NULL) {
        fprintf(stderr, "Failed to allocate memory for image\n");
        return (-1);
    }

    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    /* Open the file */
    char fname[256];
    sprintf(fname, "output/i_%04d_output.ppm", counter);
    if ((fptr = fopen(fname, "w")) == NULL) {
        fprintf(stderr, "Failed to open file for window dump\n");
        return (FALSE);
    }

    /* Copy the image into our buffer */
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);

    /* Write the raw file */
    fprintf(fptr,"P6\n%d %d\n255\n",width,height); //for ppm
    for (j = height - 1; j >= 0; j--) {
        for (i = 0; i < width; i++) {
            fputc(image[3 * j * width + 3 * i + 0], fptr);
            fputc(image[3 * j * width + 3 * i + 1], fptr);
            fputc(image[3 * j * width + 3 * i + 2], fptr);
        }
    }
    fclose(fptr);


    /* Clean up */
    counter++;
    free(image);
    return (TRUE);
}

