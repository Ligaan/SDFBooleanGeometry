// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "file_dialog_open.h"
#include <cstdio>
#include <cstring>

#ifdef _WIN32
  #include <windows.h>
  #undef max
  #undef min

  #include <Commdlg.h>
#endif

IGL_INLINE std::string igl::file_dialog_open()
{
  const int FILE_DIALOG_MAX_BUFFER = 1024;
  char buffer[FILE_DIALOG_MAX_BUFFER];
  buffer[0] = '\0';
  buffer[FILE_DIALOG_MAX_BUFFER - 1] = 'x'; // Initialize last character with a char != '\0'

#ifdef __APPLE__
  // For apple use applescript hack
  FILE * output = popen(
    "osascript -e \""
    "   tell application \\\"System Events\\\"\n"
    "           activate\n"
    "           set existing_file to choose file\n"
    "   end tell\n"
    "   set existing_file_path to (POSIX path of (existing_file))\n"
    "\" 2>/dev/null | tr -d '\n' ","r");
  if (output)
  {
    auto ret = fgets(buffer, FILE_DIALOG_MAX_BUFFER, output);
    if (ret == NULL || ferror(output))
    {
      // I/O error
      buffer[0] = '\0';
    }
    if (buffer[FILE_DIALOG_MAX_BUFFER - 1] == '\0')
    {
      // File name too long, buffer has been filled, so we return empty string instead
      buffer[0] = '\0';
    }
  }
#elif defined _WIN32

  // Use native windows file dialog box
  // (code contributed by Tino Weinkauf)

  OPENFILENAME ofn;       // common dialog box structure
  wchar_t szFile[260] = L""; // Wide-character buffer
  // Wide-character filter: pairs of description and pattern, double-null terminated
  wchar_t filter[] = L"All Files (*.*)\0*.*\0OFF Files (*.off)\0*.off\0OBJ Files (*.obj)\0*.obj\0";

  // Initialize OPENFILENAME
  ZeroMemory(&ofn, sizeof(ofn));
  ofn.lStructSize = sizeof(ofn);
  ofn.hwndOwner = NULL;
  ofn.lpstrFile = szFile;
  // Set lpstrFile[0] to '\0' so that GetOpenFileName does not
  // use the contents of szFile to initialize itself.
  ofn.lpstrFile[0] = '\0';
  ofn.nMaxFile = sizeof(szFile);
  ofn.lpstrFilter = filter;
  ofn.nFilterIndex = 1;
  ofn.lpstrFileTitle = NULL;
  ofn.nMaxFileTitle = 0;
  ofn.lpstrInitialDir = NULL;
  ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

  // Display the Open dialog box.
  int pos = 0;
  if (GetOpenFileName(&ofn)==TRUE)
  {
    while(ofn.lpstrFile[pos] != '\0')
    {
      buffer[pos] = (char)ofn.lpstrFile[pos];
      pos++;
    }
  }
  buffer[pos] = 0;
#else

  // For linux use zenity
  FILE * output = popen("/usr/bin/zenity --file-selection","r");
  if (output)
  {
    auto ret = fgets(buffer, FILE_DIALOG_MAX_BUFFER, output);
    if (ret == NULL || ferror(output))
    {
      // I/O error
      buffer[0] = '\0';
    }
    if (buffer[FILE_DIALOG_MAX_BUFFER - 1] == '\0')
    {
      // File name too long, buffer has been filled, so we return empty string instead
      buffer[0] = '\0';
    }
  }

  // Replace last '\n' by '\0'
  if(strlen(buffer) > 0)
  {
    buffer[strlen(buffer)-1] = '\0';
  }

#endif
  return std::string(buffer);
}
