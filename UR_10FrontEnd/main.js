const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs');

function createWindow () {
  const win = new BrowserWindow({
    width: 800,
    height: 600,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false
    }
  });

  win.maximize(); // Ensures fullscreen on open
  win.loadFile('index.html');
  win.removeMenu();
}

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});

ipcMain.on('save-xyz-data', async (event, data) => {
  try {
    const filePath = path.join(__dirname, 'coordinates.csv');

    fs.writeFile(filePath, data, (err) => {
      if (err) {
        console.error('Failed to save file:', err);
        event.reply('save-xyz-response', 'Failed to save file');
      } else {
        console.log('Points saved to:', filePath);
        event.reply('save-xyz-response', 'Points saved successfully!');
      }
    });
  } catch (error) {
    console.error('Error in saving file:', error);
    event.reply('save-xyz-response', 'Error saving points');
  }
});
