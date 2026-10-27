# smc3-scope - dashboard, tuning, auto-tune and diagnostics page for the SMC3_R4 motion rig controller.
# Standard-library Python only, so the image is tiny and needs no package installs.
FROM python:3.12-alpine
LABEL org.opencontainers.image.title="smc3-scope" org.opencontainers.image.source="https://github.com/winreboot/smc3-r4"
WORKDIR /app
COPY smc3_scope.py ./smc3_scope.py
ENV SMC3_BIND=0.0.0.0 SMC3_DATA_DIR=/data
VOLUME /data
CMD ["python", "-u", "smc3_scope.py"]
