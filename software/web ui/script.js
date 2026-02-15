const API_URL = "http://127.0.0.1:5000/data";


let rawData = [];
let filteredData = [];
let refreshTimerId = null;
let chart = null;

const tableBody = document.getElementById("data-table-body");
const refreshSelect = document.getElementById("refresh-interval-select");
const chartCanvas = document.getElementById('water-chart');
const dateFrom = document.getElementById('dateFrom');
const dateUntil = document.getElementById('dateUntil');

async function getData() {
    const response = await fetch(API_URL);
    const json = await response.json();
    rawData = json
      .map(r => ({
        timestamp: Number(r.timestamp),
        value: Number(r.value)
      }));

    const from = dateFrom.value ? new Date(dateFrom.value).getTime() / 1000 : -Infinity;
    const until = dateUntil.value ? new Date(dateUntil.value).getTime() / 1000 : Infinity;

    filteredData = rawData.filter(r => {
      return r.timestamp >= from && r.timestamp <= until;
    })
      
    renderTable();
    renderChart();
}

function renderTable() {
  tableBody.innerHTML = "";
  for (const item of filteredData) {
    const tr = document.createElement("tr");

    const dateCell = document.createElement("td");
    const date = new Date(item.timestamp * 1000);
    dateCell.textContent = date.toLocaleString();

    const valueCell = document.createElement("td");
    valueCell.textContent = item.value;

    tr.appendChild(dateCell);
    tr.appendChild(valueCell);
    tableBody.appendChild(tr);
  }
}

function setAutoRefresh() {
  const interval = parseInt(refreshSelect.value, 10);
  if (refreshTimerId) {
    clearInterval(refreshTimerId);
    refreshTimerId = null;
  }
  if (interval > 0) {
    refreshTimerId = setInterval(getData, interval);
  }
}

function renderChart() {

  const labels = filteredData.map(item =>
    new Date(item.timestamp * 1000).toLocaleTimeString()
  );
  const values = filteredData.map(item => item.value);

  if (!chart) {
    chart = new Chart(chartCanvas, {
      type: 'line',
      data: {
        labels: labels,
        datasets: [{
          label: 'Wasserstand in m³',
          data: values,
          borderWidth: 1,
          responsive: true,
          maintainAspectRatio: false,
        }]
      },
      options: {
        scales: {
          x: {
            ticks: {
              color: "white", //white
              maxRotation: 0,
              autoSkip: true,
              maxTicksLimit: 6,
              font: {
                size: 11
              }
            }
          },
          y: {
            beginAtZero: true,
            grid: { color: "#1f2937" }, //"#1f2937"
            ticks: {
              color: "white", //white
              font: {
                size: 11
              }
            }
          }
        },
        plugins: {
            tooltip: {
              mode: "index",
              intersect: false
            },
            legend: {
              labels: {
                color: "white", //white
                font: { size: 11 }
              }
            }
          }
      }
    });
  } else {
    chart.data.labels = labels;
    chart.data.datasets[0].data = values;
    chart.update();
  }
}

function clearDates() {
  dateFrom.value = '';
  dateUntil.value = '';
  getData();
}


refreshSelect.addEventListener("change", setAutoRefresh);

getData();